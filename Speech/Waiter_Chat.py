import datetime  # Times for orders
import tkinter as tk  # GUI
from tkinter import scrolledtext, messagebox, simpledialog
import threading
import whisper  # Speech-to-text
import numpy as np
import pyttsx3  # Text-to-Speech
import pyaudio
import os
import configparser  # Reading the config file
import string
import re
from collections import defaultdict  # For easily counting item quantities


class RobotWaiter:

    # Dictionary to map spoken number words to their integer equivalents
    NUMBER_WORDS = {
        'a': 1, 'an': 1, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
        'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
        'eleven': 11, 'twelve': 12, 'thirteen': 13, 'fourteen': 14, 'fifteen': 15,
        'sixteen': 16, 'seventeen': 17, 'eighteen': 18, 'nineteen': 19, 'twenty': 20
    }

    
    def __init__(self, root):
        
        # Main window
        self.root = root  
        self.root.title("Robot Waiter")
        # Handle window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close) 
        self.engine_lock = threading.Lock()  # Lock for when the tts is running to avoid crashes
        
        # Color scheme
        self.colors = {
            'primary': '#4A6FA5', 
            'secondary': '#166088', 
            'accent': '#4FC3F7',
            'background': '#F5F7FA', 
            'text': '#333333', 
            'success': '#4CAF50',
            'warning': '#FF9800', 
            'danger': '#F44336', 
            'card': '#FFFFFF',
            'border': '#E0E0E0'
        }
        
        # Configure the root window (backround & fullscreen)
        self.root.configure(bg=self.colors['background'])
        self.root.attributes('-fullscreen', True)
        
        # State variables for the application
        self.is_speaking = False  # Flag for if TTS is active
        self.shutting_down = False  # Flag for if the program is closing
        self.all_orders = []  # Stores all confirmed items as a list (e.g., ['coke', 'burger', 'burger'])
        
        # Initialize audio components
        self.audio = pyaudio.PyAudio()
        self.model = whisper.load_model("base")
        
        # Read configuration from config.txt
        self.config = self.read_config()
        self.table_count = self.config.getint('Number of Tables', 'count', fallback=10)
        self.table_count = max(1, min(60, self.table_count)) # Min and max amount of tables
        # Load menu items from config, ensuring they're lowercase for matching
        self.menu = [item.strip().lower() for item in 
                    self.config.get('Menu', 'items', fallback='pizza,chicken,chips,coke,burger,salad,ice cream').split(',')]
        
        # Lists of words considered as affirmative or negative responses
        self.yes_similes = ['yes', 'yeah', 'yep', 'sure', 'correct', 'right', 'affirmative', 'ok', 'okay']
        self.no_similes = ['no', 'nope', 'nah', 'no thank you', 'done', 'finished', 'negative', 'incorrect', 'wrong']
        
        # Stores items parsed in the current turn of ordering (before confirmation)
        self.current_order = []
        # Check if speech is recording
        self.is_recording = False
        self.selected_table = None  # Stores the currently selected table number
        self.engine = None  # TTS engine instance
        
        # Initial GUI setup
        self.show_table_selection()  # Display table selection screen
        # Escape key to exit exit fullscreen
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))


    # Called when the main window is closed
    def on_close(self):

        self.shutting_down = True  # Set shutdown flag
        # Stop TTS engine if it's running
        if hasattr(self, 'engine') and self.engine:
            try:
                self.engine.stop()
            except RuntimeError: 
                pass  # Ignore error if engine is already stopped or in a bad state

        # Terminate PyAudio instance
        if hasattr(self, 'audio'): 
            self.audio.terminate()
        # Destroy the Tkinter root window
        if self.root.winfo_exists():
            self.root.destroy()
    

    # Removes punctuation from text and converts it to lowercase
    def remove_punctuation(self, text):

        translator = str.maketrans('', '', string.punctuation)
        text = text.translate(translator)  # Apply change
        return re.sub(' +', ' ', text).strip().lower()  # Remove extra spaces, strip, and lowercase


    # Check if the text is affirmative
    def is_affirmative(self, text):

        # Cleans the text and checks
        clean_text = self.remove_punctuation(text)
        return any(word in clean_text for word in self.yes_similes)


    # Check if the text is negative
    def is_negative(self, text):

        # Cleans the text and checks
        clean_text = self.remove_punctuation(text)
        return any(word in clean_text for word in self.no_similes)


    # Disables GUI user input (e.g. while TTS is speaking or processing)
    def disable_input(self):

        self.is_speaking = True  # Set speaking flag
        if not self.root.winfo_exists(): 
            return # Don't proceed if root window is gone
        
        self.root.config(cursor="watch")  # Change cursor to 'watch'
        # Temporarily unbind click and key events to prevent interaction
        self.root.bind("<Button-1>", lambda e: "break") 
        self.root.bind("<Key>", lambda e: "break")

        # Disable specific buttons if they exist
        if hasattr(self, 'record_btn') and self.record_btn.winfo_exists():
            self.record_btn.config(state=tk.DISABLED)

        if hasattr(self, 'user_input') and self.user_input.winfo_exists():
            self.user_input.config(state=tk.DISABLED)

        if hasattr(self, 'send_btn') and self.send_btn.winfo_exists():
             self.send_btn.config(state=tk.DISABLED)
        
        # Disable buttons within confirmation or management
        for frame_name in ['confirm_frame', 'management_frame']:
            if hasattr(self, frame_name):

                frame = getattr(self, frame_name)
                if frame and frame.winfo_exists():

                    for widget in frame.winfo_children():
                        if isinstance(widget, tk.Button):

                            widget.config(state=tk.DISABLED)


    # Enables user input elements in the GUI
    def enable_input(self):

        self.is_speaking = False  # Clear speaking flag
        if not self.root.winfo_exists(): 
            return # Don't proceed if root window is gone
        
        self.root.config(cursor="")  # Reset cursor
        # Re-enable click and key events
        self.root.unbind("<Button-1>")
        self.root.unbind("<Key>")

        # Enable specific buttons if they exist
        if hasattr(self, 'record_btn') and self.record_btn.winfo_exists():
            self.record_btn.config(state=tk.NORMAL)

        if hasattr(self, 'user_input') and self.user_input.winfo_exists():
            self.user_input.config(state=tk.NORMAL)
            
        if hasattr(self, 'send_btn') and self.send_btn.winfo_exists():
            self.send_btn.config(state=tk.NORMAL)

        # Enable buttons within confirmation or management
        for frame_name in ['confirm_frame', 'management_frame']:
            if hasattr(self, frame_name):

                frame = getattr(self, frame_name)
                if frame and frame.winfo_exists():
                    
                    for widget in frame.winfo_children():
                        if isinstance(widget, tk.Button):

                            widget.config(state=tk.NORMAL)


    # Speaks the given text using the TTS engine
    def speak(self, text):

        # If shutting down or TTS engine not initialized, skip speaking
        if self.shutting_down or not self.engine: 
            if not self.engine and not self.shutting_down: # If TTS not available

                print(f"TTS Skipped (engine not available): {text}")
                display_text = text.replace('Waiter: ', '') if text.startswith('Waiter: ') else text
                self.update_chat(f"Waiter (TTS not available): {display_text}") 

            if not self.shutting_down and self.root.winfo_exists(): # Still enable input if TTS skipped

                self.root.after(0, self.enable_input) # Ensure input is re-enabled

            return 
        
        self.disable_input()  # Disable GUI input while speaking
        # Define the speaking task to be run in a separate thread
        def _speak():

            try:

                with self.engine_lock:  # Acquire lock for TTS engine
                    self.engine.say(text)
                    self.engine.runAndWait()  # Block until speaking is finished

            except Exception as e:
                print(f"TTS Error: {e}")  # Log any TTS errors

            finally:
                # Re-enable input after speaking, if not shutting down
                if not self.shutting_down and self.root.winfo_exists(): 
                    self.root.after(0, self.enable_input)

        # Start the speaking task in a new thread
        threading.Thread(target=_speak, daemon=True).start()


    # Reads configuration from 'config.txt'
    def read_config(self):

        config = configparser.ConfigParser()  # Create a config parser instance

        # Default values, created if the config file isn't found
        if not os.path.exists("config.txt"):

            with open("config.txt", "w") as f:
                f.write("[Number of Tables]\ncount = 10\n\n[Menu]\nitems = pizza, chicken, chips, coke, burger, salad, ice cream\n")

        try:

            config.read("config.txt")  # Read the config file
            # Ensure default sections/values if they are missing
            if 'Number of Tables' not in config: config['Number of Tables'] = {'count': '10'}
            if 'Menu' not in config: config['Menu'] = {'items': 'pizza, chicken, chips, coke, burger, salad, ice cream'}

            return config
        
        except Exception as e:

            # Show warning and use defaults if there's an error reading the config
            messagebox.showwarning("Config Error", f"Error reading config.txt\nUsing default values\nError: {str(e)}")
            config['Number of Tables'] = {'count': '10'}
            config['Menu'] = {'items': 'pizza, chicken, chips, coke, burger, salad, ice cream'}

            return config
        

    # Initializes the TTS engine
    def initialize_tts(self):

        try:

            self.engine = pyttsx3.init()  # Initialize
            if self.engine: 
                 self.engine.setProperty('rate', 220)  # Set speech rate
            else:
                # For 'inbetween' cases where the TTS isn't working properly but hasn't neceserally failed
                print("TTS Engine failed to initialize.") 
                messagebox.showerror("TTS Error", "Text-to-speech engine could not be initialized.")

        except Exception as e:

            print(f"TTS Initialization Error: {e}") 
            messagebox.showerror("TTS Error", f"Text-to-speech engine failed: {e}")
            # Ensure engine is None if initialization fails
            self.engine = None


    # Button
    def create_button(self, parent, text, command, bg_color, fg_color='white', **kwargs):

        # Button style
        btn = tk.Button(parent, text=text, command=command, bg=bg_color, fg=fg_color,
                        activebackground=self.adjust_color(bg_color, -20), activeforeground=fg_color,
                        borderwidth=0, highlightthickness=0, relief='flat', font=('Arial', 12, 'bold'),
                        padx=15, pady=8, **kwargs)
        
        # Bind mouse enter and leave events for hover effect
        btn.bind("<Enter>", lambda e, b=btn, color=bg_color: b.config(bg=self.adjust_color(color, -10))) # Darken on hover
        btn.bind("<Leave>", lambda e, b=btn, color=bg_color: b.config(bg=color)) # Restore original color
        return btn


    # Adjusts a hex color (lighten or darken)
    def adjust_color(self, color, amount):

        from colorsys import rgb_to_hls, hls_to_rgb  # Import locally

        color = color.lstrip('#')  # Remove '#'
        rgb = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))  # Convert hex to RGB tuple
        h, l, s = rgb_to_hls(*[x/255.0 for x in rgb])  # Convert RGB to HLS
        l = max(0, min(1, l + amount/100.0))  # Adjust lightness (clamped between 0 and 1)
        rgb_adjusted = tuple(int(x*255) for x in hls_to_rgb(h, l, s))  # Convert back to RGB
        # Format as hex string
        return f'#{rgb_adjusted[0]:02x}{rgb_adjusted[1]:02x}{rgb_adjusted[2]:02x}'
        

    # Displays the first screen for table selection
    def show_table_selection(self):

        self.clear_gui()  # Clear any existing widgets

        # Main container for table selection
        container = tk.Frame(self.root, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=50, pady=50)

        # Header text
        header = tk.Frame(container, bg=self.colors['background'])
        header.pack(fill=tk.X, pady=(0, 30))
        tk.Label(header, text="Please select your table number:", font=('Arial', 18, 'bold'), fg=self.colors['text'], bg=self.colors['background']).pack()

        # Keep table buttons in a grid
        table_frame = tk.Frame(container, bg=self.colors['background'])
        table_frame.pack(expand=True)
        # Maximum 10 columns for table buttons
        cols = min(10, self.table_count)

        # Create a button for each table
        for i in range(1, self.table_count + 1):
            btn = self.create_button(table_frame, text=str(i), command=lambda num=i: self.set_table(num),
                                           bg_color=self.colors['primary'], fg_color='white')
            btn.grid(row=(i-1)//cols, column=(i-1)%cols, padx=10, pady=10, ipadx=20, ipady=15) # Arrange in grid
        
        # Footer with an exit button
        footer = tk.Frame(container, bg=self.colors['background'])
        footer.pack(fill=tk.X, pady=(30, 0))
        self.create_button(footer, text="Exit", command=self.on_close,
                                   bg_color=self.colors['danger'], fg_color='white').pack(side=tk.RIGHT)


    # Sets the selected table number and move to the main screen
    def set_table(self, table_num):

        # Store selected table
        self.selected_table = table_num
        # Initialize TTS engine now that a table is selected
        self.initialize_tts()
        # Announce table selection
        self.speak(f"Table {table_num} selected")
        # Clear screen
        self.clear_gui()
        # Create the main ordering interface
        self.create_main_screen()
        # Speak the welcome message
        self.speak_welcome()


    # Clears all widgets from the window
    def clear_gui(self):

        for widget in self.root.winfo_children():
            widget.destroy()


    # Creates the main user interface for ordering
    def create_main_screen(self):

        # Main container for the ordering screen
        container = tk.Frame(self.root, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=50, pady=50)
        # Header displaying the current table number
        header = tk.Frame(container, bg=self.colors['background'])
        header.pack(fill=tk.X, pady=(0, 20))
        tk.Label(header, text=f"Table: {self.selected_table}", font=('Arial', 16, 'bold'),
                   fg=self.colors['primary'], bg=self.colors['background']).pack(side=tk.LEFT)
        
        # Frame for the chat display area
        chat_frame = tk.Frame(container, bg=self.colors['background'])
        chat_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 20))
        # Scrolled text widget for chat history
        self.chat_display = scrolledtext.ScrolledText(
            chat_frame, wrap=tk.WORD, font=('Arial', 12), bg=self.colors['card'], fg=self.colors['text'],
            bd=2, relief=tk.GROOVE, highlightthickness=0, padx=20, pady=20)
        self.chat_display.pack(fill=tk.BOTH, expand=True)
        # Define tags for styling user and bot messages
        self.chat_display.tag_config('user', foreground=self.colors['primary'])
        self.chat_display.tag_config('bot', foreground=self.colors['secondary'])
        self.chat_display.config(state=tk.DISABLED)  # Make chat display read only initially
        
        # Frame for user input (text entry and send button)
        input_frame = tk.Frame(container, bg=self.colors['background'])
        input_frame.pack(fill=tk.X, pady=(0, 20))
        # Text entry field for user input
        self.user_input = tk.Entry(
            input_frame, font=('Arial', 12), bg=self.colors['card'], fg=self.colors['text'],
            bd=2, relief=tk.GROOVE, highlightthickness=0, insertbackground=self.colors['text'])
        self.user_input.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10), pady=10)
        self.user_input.bind("<Return>", lambda e: self.process_text_input())  # Enter key to send input
        
        # Send button for text input
        self.send_btn = self.create_button(
            input_frame, text="Send", command=self.process_text_input,
            bg_color=self.colors['accent'], fg_color='white')
        self.send_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Frame for action buttons
        button_frame = tk.Frame(container, bg=self.colors['background'])
        button_frame.pack(fill=tk.X)
        # Record order button
        self.record_btn = self.create_button(
            button_frame, text="Voice Control", command=self.start_recording,
            bg_color=self.colors['primary'], fg_color='white')
        self.record_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Status bar label at the bottom
        self.status = tk.Label(
            container, text="Ready", bd=0, relief=tk.FLAT, anchor=tk.W, font=('Arial', 10),
            fg=self.colors['text'], bg=self.colors['background'])
        self.status.pack(fill=tk.X, pady=(10, 0))


    # Speaks the initial welcome message and lists menu items
    def speak_welcome(self):

        welcome_msg = f"Welcome table {self.selected_table}! Our menu has: " + ", ".join(self.menu)
        self.speak(welcome_msg)


    # Starts the audio recording process
    def start_recording(self):
        
        # Prevent multiple recordings
        if self.is_recording: 
            return
        
        self.is_recording = True
         # Disable record button during recording
        if hasattr(self, 'record_btn') and self.record_btn.winfo_exists(): 
            self.record_btn.config(state=tk.DISABLED)
        
        self.set_status("Preparing to record...")
        # Start recording and processing in a new thread to keep GUI responsive
        threading.Thread(target=self.record_and_process, daemon=True).start()


    # Records audio from the microphone and processes it
    def record_and_process(self):

        try:

            # Prompt user to speak
            self.set_status("Recording... Speak now!")
            self.speak("Speak now")  
            
            # Audio recording parameters
            FORMAT, CHANNELS, RATE, CHUNK, RECORD_SECONDS = pyaudio.paInt16, 1, 16000, 1024, 10 
            
            # Open audio stream for recording
            stream = self.audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
            frames = []  # List to store audio frames
            # Record audio
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                
                # Allow stopping recording (not fully implemented)
                if not self.is_recording: 
                    break
                # Read audio chunk
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
            
            # Stop and close the audio stream
            stream.stop_stream()
            stream.close()
            
            # Process recorded audio
            if frames:

                audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)  # Convert frames to numpy array
                self.process_audio_data(audio_data)

            else:
                
                self.set_status("No audio recorded.") # Update status if no audio captured
            
        except Exception as e:

            self.set_status(f"Recording Error: {str(e)}")
            self.speak("Sorry, there was an error with the recording")
            self.update_chat(f"System Error during recording: {str(e)}")

        finally:

            self.is_recording = False  # Reset recording flag
            # Re-enable record button if GUI still exists
            if self.root.winfo_exists(): 
                 self.root.after(0, lambda: self.record_btn.config(state=tk.NORMAL) if hasattr(self, 'record_btn') and self.record_btn.winfo_exists() else None)
            # Reset status message
            self.set_status("Ready")


    # Processes the recorded audio data using Whisper for STT
    def process_audio_data(self, audio_data):

        try:

            self.set_status("Transcribing audio...")
            # Normalize audio data for Whisper
            audio_np = audio_data.astype(np.float32) / 32768.0
            # Transcribe audio
            result = self.model.transcribe(audio_np, language='en')
            # Get transcribed text
            order_text = result['text'].strip()
            # Display transcribed text
            self.update_chat(f"You (audio): {order_text}", is_user=True)
            # Process the transcribed order text
            self.process_order_text(order_text)

        except Exception as e:

            self.set_status(f"Transcription Error: {str(e)}")
            self.speak("Sorry, I couldn't process that order")
            self.update_chat(f"System Error during transcription: {str(e)}")


    # Processes text input from the user entry box
    def process_text_input(self):

        # Get text from entry field
        text = self.user_input.get().strip()
        # Ignore empty input
        if not text: 
            return
        # Display user's text input
        self.update_chat(f"You: {text}", is_user=True)
        # Clear the entry field
        self.user_input.delete(0, tk.END)
        # Process the order text
        self.process_order_text(text)


    # Helper to parse a single word as a quantity (e.g., "two" -> 2, "3" -> 3)
    def _parse_word_as_quantity(self, word_token):

        # Convert to lowercase for matching
        word_token_lower = word_token.lower()
        # Check against predefined number word
        if word_token_lower in self.NUMBER_WORDS:
            return self.NUMBER_WORDS[word_token_lower]
        try:

            num = int(word_token_lower)  # Try converting to integer
            if 1 <= num <= 999:  # Accept quantities between 1 and 999
                return num
            
        except ValueError:
            
            pass  # Not a digit

        return None  # Return None if not a recognized quantity


    # Helper to find the menu item name from a word, checking singular/plural forms
    def _get_menu_item_from_word(self, word_to_check):

        # 1. Exact match with menu item
        if word_to_check in self.menu:
            return word_to_check
        
        # 2. If word_to_check seems plural (ends in 's', not 'ss', not 'chips'), try its singular form
        if word_to_check.endswith('s') and not word_to_check.endswith('ss') and word_to_check != "chips":
            # Remove 's'
            singular_form = word_to_check[:-1]
            if singular_form in self.menu:
                # Return singular form
                return singular_form
        
        # 3. If word_to_check seems singular (and not 'chips'), try its plural form
        if not word_to_check.endswith('s') and word_to_check != "chips":
            # Add 's'
            plural_form = word_to_check + 's'
            if plural_form in self.menu:
                # Return plural form (if that's how it's in menu)
                return plural_form
        
        return None # No match found


    # Processes the order text (from STT or text input) to identify items and quantities
    def process_order_text(self, text):

        # Clean and normalize text
        clean_text = self.remove_punctuation(text)
        # Split text into words
        words = clean_text.split()
        
        # If in confirmation mode, handle yes/no responses first
        if hasattr(self, 'confirm_frame') and self.confirm_frame.winfo_exists() and self.confirm_frame.winfo_ismapped():

            if self.is_affirmative(clean_text):
                self.confirm_order_yes()
                return
            
            elif self.is_negative(clean_text):
                self.confirm_order_no()
                return
        
        # Stores items identified in this interaction
        parsed_items_for_current_turn = []
        i = 0
        while i < len(words):
            word = words[i]
            quantity = self._parse_word_as_quantity(word)  # Try to parse current word as quantity

            # If a quantity is found and there's a next word
            if quantity is not None and i + 1 < len(words):
                next_word_candidate = words[i+1]  # The word following the quantity
                # Try to match the next word (and its variations) to a menu item
                menu_item_name = self._get_menu_item_from_word(next_word_candidate)
                
                if menu_item_name:

                    # Add the item 'quantity' times to the list
                    parsed_items_for_current_turn.extend([menu_item_name] * quantity)
                    i += 2  # Move past quantity and item words

                    continue  # Continue to next part of the input
            
            # If not a (quantity, item) pair, check if the current word itself is a menu item
            menu_item_name_standalone = self._get_menu_item_from_word(word)
            if menu_item_name_standalone:
                parsed_items_for_current_turn.append(menu_item_name_standalone)  # Add with default quantity 1
            
            i += 1  # Move to the next word

        # If no items were recognized
        if not parsed_items_for_current_turn:

            response = "I didn't recognize any menu items. Our menu has: " + ", ".join(self.menu)
            self.speak(response)
            self.update_chat(f"Waiter: {response}")

            return

        self.current_order = parsed_items_for_current_turn  # Update the order for this turn
        
        # Prepare display string for confirmation
        item_counts = defaultdict(int)  # Count occurrences of each item
        for item in self.current_order:
            item_counts[item] += 1
        
        display_items_list = []
        for item, qty in item_counts.items():
            
            display_name = item  # Start with canonical name
            if qty > 1: # If quantity is more than 1, try to make it plural for display
                if item == "chips":  # 'chips' is already plural
                    display_name = f"{qty} {item}"

                elif item.endswith('s'): # e.g. 'nachos'
                     display_name = f"{qty} {item}"

                else: # simple pluralization by adding 's'
                    display_name = f"{qty} {item}s" 

            else: # quantity is 1
                display_name = f"{item}"

            display_items_list.append(display_name)
        
        order_string = ", ".join(display_items_list)
        response = f"I heard: {order_string}. Is this correct?"
        self.speak(response)
        self.update_chat(f"Waiter: {response}")
        # Show Yes/No buttons for confirmation
        self.show_confirmation_buttons()


    # Adds a message to the chat display area
    def update_chat(self, message, is_user=False):

        # Ensure GUI elements exist before trying to update
        if not self.root.winfo_exists() or not hasattr(self, 'chat_display') or not self.chat_display.winfo_exists(): 
            return
        
        # Enable editing
        self.chat_display.config(state=tk.NORMAL)
        # Apply user/bot tag
        tag = "user" if is_user else "bot"
        # Insert message
        self.chat_display.insert(tk.END, f"{message}\n", tag)
        # Disable editing
        self.chat_display.config(state=tk.DISABLED)
        # Scroll to the end
        self.chat_display.see(tk.END)


    # Updates the text in the status bar
    def set_status(self, message):

        if not self.root.winfo_exists() or not hasattr(self, 'status') or not self.status.winfo_exists(): 
            return
        self.status.config(text=message)
    

    # Displays Yes/No confirmation buttons
    def show_confirmation_buttons(self):
        
        # Remove any existing confirmation buttons
        self.remove_confirmation_buttons()
        if not self.root.winfo_exists(): 
            return 
        
        self.confirm_frame = tk.Frame(self.root, bg=self.colors['background'])
        self.confirm_frame.pack(pady=10)
        
        self.create_button(self.confirm_frame, text="Yes", command=self.confirm_order_yes, bg_color=self.colors['success'], fg_color='white').pack(side=tk.LEFT, padx=10)
        self.create_button(self.confirm_frame, text="No", command=self.confirm_order_no, bg_color=self.colors['danger'], fg_color='white').pack(side=tk.LEFT, padx=10)


    # Removes the Yes/No confirmation buttons
    def remove_confirmation_buttons(self):

        if hasattr(self, 'confirm_frame') and self.confirm_frame and self.confirm_frame.winfo_exists():

            self.confirm_frame.destroy()
            # Check again before delattr as destroy might be deferred
            if hasattr(self, 'confirm_frame'):

                delattr(self, 'confirm_frame')


    # Handles the "Yes" confirmation
    def confirm_order_yes(self):

        self.update_chat("You: Yes", is_user=True)
        # If there are items in the current turn's order
        if self.current_order:
            
            # Add them to the main order list
            self.all_orders.extend(self.current_order)  

            # Prepare display string for the confirmed order
            item_counts = defaultdict(int)
            for item_name in self.current_order:
                item_counts[item_name] += 1
            
            display_items_list = []
            for item, qty in item_counts.items(): 

                display_name = item
                if qty > 1:
                    if item == "chips": 
                        display_name = f"{qty} {item}"

                    elif item.endswith('s'):
                        display_name = f"{qty} {item}"

                    else:
                        display_name = f"{qty} {item}s"

                else:
                    display_name = f"{item}"

                display_items_list.append(display_name)
            
            order_string = ", ".join(display_items_list)
            response = f"Order confirmed! You ordered: {order_string}"
            self.speak(response)
            self.update_chat(f"Waiter: {response}")
            # Show next set of options (View/Edit, Continue, Finish)
            self.show_order_management()
        
        # Remove Yes/No buttons
        self.remove_confirmation_buttons()
        # Clear current turn's order
        self.current_order = []

    # Handles the "No" rejection from the user
    def confirm_order_no(self):

        self.update_chat("You: No", is_user=True)
        self.remove_confirmation_buttons()
        response = "My apologies. Let's try again. What would you like to order?"
        self.speak(response)
        self.update_chat(f"Waiter: {response}")
        # Clear rejected items
        self.current_order = []


    # Displays order management buttons (View/Edit, Continue, Finish)
    def show_order_management(self):

        # Ensure Yes/No buttons are gone
        self.remove_confirmation_buttons()
        # Remove any existing management buttons
        self.remove_management_buttons()
        
        # Don't proceed if window is gone
        if not self.root.winfo_exists(): 
            return
        
        self.management_frame = tk.Frame(self.root, bg=self.colors['background'])
        self.management_frame.pack(pady=10)
        
        self.create_button(self.management_frame, text="View/Edit Orders", command=self.show_full_order, bg_color=self.colors['warning'], fg_color='black').pack(side=tk.LEFT, padx=10)
        self.create_button(self.management_frame, text="Continue Ordering", command=self.continue_ordering, bg_color=self.colors['success'], fg_color='white').pack(side=tk.LEFT, padx=10)
        self.create_button(self.management_frame, text="Finish & Pay", command=self.finish_order, bg_color=self.colors['danger'], fg_color='white').pack(side=tk.LEFT, padx=10)


    # Removes the order management buttons
    def remove_management_buttons(self):

        if hasattr(self, 'management_frame') and self.management_frame and self.management_frame.winfo_exists():

            self.management_frame.destroy()
            # Check again before delattr
            if hasattr(self, 'management_frame'):
                 
                 delattr(self, 'management_frame')


    # Displays a new window showing the full current order with options to edit
    def show_full_order(self):

        # If no items ordered yet
        if not self.all_orders:

            msg = "You haven't ordered anything yet."
            self.speak(msg)
            self.update_chat(f"Waiter: {msg}")

            return
            
        # Create a new top-level window for order display
        order_window = tk.Toplevel(self.root)
        order_window.title("Your Current Order")
        order_window.configure(bg=self.colors['background'])
        
        # Center the new window
        window_width, window_height = 500, 400
        x = (self.root.winfo_screenwidth() - window_width) // 2
        y = (self.root.winfo_screenheight() - window_height) // 2
        order_window.geometry(f"{window_width}x{window_height}+{x}+{y}")
        # Keep this window on top of the main one
        order_window.transient(self.root)
        # Make this window modal (blocks interaction with main window)
        order_window.grab_set()

        # Container for the order display window
        container = tk.Frame(order_window, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
        
        tk.Label(container, text="Your Current Order", font=('Arial', 16, 'bold'), fg=self.colors['text'], bg=self.colors['background']).pack(pady=(0, 15))
        
        # Frame for the listbox and scrollbar
        list_frame = tk.Frame(container, bg=self.colors['background'])
        list_frame.pack(fill=tk.BOTH, expand=True)
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Listbox to display ordered items
        # # exportselection=False allows selection in multiple listboxes if needed
        self.order_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, bg=self.colors['card'], fg=self.colors['text'], bd=2, relief=tk.GROOVE, highlightthickness=0, font=('Arial', 12), selectbackground=self.colors['accent'], exportselection=False)
        self.order_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        # Link scrollbar to listbox
        scrollbar.config(command=self.order_listbox.yview)
        
        # Populate listbox with items from self.all_orders (which is a flat list)
        for idx, item in enumerate(self.all_orders, 1): 
            self.order_listbox.insert(tk.END, f"{idx}. {item}")
        
        # Frame for buttons (Remove, Swap, Close)
        btn_frame = tk.Frame(container, bg=self.colors['background'])
        btn_frame.pack(fill=tk.X, pady=(15, 0))
        
        self.create_button(btn_frame, text="Remove Selected", command=self.remove_selected_item, bg_color=self.colors['danger'], fg_color='white').pack(side=tk.LEFT, padx=5)
        self.create_button(btn_frame, text="Swap Selected", command=self.swap_selected_item, bg_color=self.colors['primary'], fg_color='white').pack(side=tk.LEFT, padx=5)
        self.create_button(btn_frame, text="Close", command=order_window.destroy, bg_color=self.colors['secondary'], fg_color='white').pack(side=tk.RIGHT, padx=5)


    # Removes the selected item from the order
    def remove_selected_item(self):

        if not hasattr(self, 'order_listbox') or not self.order_listbox.winfo_exists(): 
            return
        # Get selected item index
        selection = self.order_listbox.curselection()
        # No item selected
        if not selection: 
            return  
            
        index_to_remove = selection[0]
        # Remove from the main order list
        removed_item = self.all_orders.pop(index_to_remove)
        
        # Refresh the listbox display
        self.order_listbox.delete(0, tk.END)  # Clear all items
        # Re populate with updated list
        for idx, item_in_list in enumerate(self.all_orders, 1):
            self.order_listbox.insert(tk.END, f"{idx}. {item_in_list}")
        
        msg = f"Removed {removed_item} from your order."
        self.speak(msg)
        self.update_chat(f"Waiter: {msg}")


    # Allows swapping the selected item with another item from the menu
    def swap_selected_item(self):

        if not hasattr(self, 'order_listbox') or not self.order_listbox.winfo_exists(): 
            return
        selection = self.order_listbox.curselection()
        if not selection: 
            return

        # Index of the item to swap in self.all_orders
        original_index = selection[0]
        # The item string
        current_item = self.all_orders[original_index]

        # Create a new Toplevel window for swapping
        swap_window = tk.Toplevel(self.root)
        swap_window.title(f"Swap {current_item}")
        swap_window.configure(bg=self.colors['background'])
        window_width, window_height = 350, 350 
        x = (self.root.winfo_screenwidth() - window_width) // 2
        y = (self.root.winfo_screenheight() - window_height) // 2
        swap_window.geometry(f"{window_width}x{window_height}+{x}+{y}")
        swap_window.transient(self.root)
        swap_window.grab_set()

        container = tk.Frame(swap_window, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
        tk.Label(container, text=f"Swap '{current_item}' with:", font=('Arial', 14), fg=self.colors['text'], bg=self.colors['background']).pack(pady=(0, 15))

        # Listbox to show available menu items for swapping
        list_frame = tk.Frame(container, bg=self.colors['background'])
        list_frame.pack(fill=tk.BOTH, expand=True)
        menu_listbox = tk.Listbox(list_frame, bg=self.colors['card'], fg=self.colors['text'], bd=2, relief=tk.GROOVE, highlightthickness=0, font=('Arial', 12), selectbackground=self.colors['accent'], exportselection=False)
        menu_listbox.pack(fill=tk.BOTH, expand=True)
        # Populate with menu items
        for item_on_menu in self.menu: 
            menu_listbox.insert(tk.END, item_on_menu)
        
        # Buttons for Swap and Cancel
        btn_frame = tk.Frame(container, bg=self.colors['background'])
        btn_frame.pack(fill=tk.X, pady=(15, 0))
        self.create_button(btn_frame, text="Swap", command=lambda: self.perform_swap(swap_window, menu_listbox, original_index, current_item), bg_color=self.colors['success'], fg_color='white').pack(side=tk.LEFT, padx=5)
        self.create_button(btn_frame, text="Cancel", command=swap_window.destroy, bg_color=self.colors['danger'], fg_color='white').pack(side=tk.RIGHT, padx=5)


    # Performs the actual item swap operation
    def perform_swap(self, window, listbox, original_index_in_all_orders, old_item_name):

        # Get selected item from the menu listbox
        swap_selection = listbox.curselection()
        if not swap_selection: 
            return
            
        # Get the new item name
        new_item_name = self.menu[swap_selection[0]]
        # Update the main order list
        self.all_orders[original_index_in_all_orders] = new_item_name
        
        # Refresh the main order listbox in the "View/Edit Orders" window
        if hasattr(self, 'order_listbox') and self.order_listbox.winfo_exists():
            self.order_listbox.delete(0, tk.END) 
            for idx, item_in_list in enumerate(self.all_orders, 1):
                self.order_listbox.insert(tk.END, f"{idx}. {item_in_list}")
        
        msg = f"Swapped {old_item_name} with {new_item_name}."
        self.speak(msg)
        self.update_chat(f"Waiter: {msg}")
        # Close the swap dialog
        window.destroy()


    # Allows the user to continue adding more items to the order
    def continue_ordering(self):
        
        # Remove View/Edit, Continue, Finish buttons
        self.remove_management_buttons()
        msg = "What else would you like to order?"
        self.speak(msg)
        self.update_chat(f"Waiter: {msg}")
        # Clear current turn's order list
        self.current_order = []


    # Finalizes the order
    def finish_order(self):

        self.remove_management_buttons() 
        self.remove_confirmation_buttons() 
        
        # If there are items in the order
        if self.all_orders:
            # Create 'orders' directory if it doesn't exist
            if not os.path.exists("orders"): 
                os.makedirs("orders")
            # Save the order
            self.save_order_to_file()
            # End the session
            self.handle_session_end()

        else: # If order is empty

            msg = "You haven't ordered anything yet. Are you sure you want to finish?"
            self.speak(msg)
            self.update_chat(f"Waiter: {msg}")
            # Ask for confirmation to finish empty order
            self.show_finish_empty_order_confirmation()


    # Shows confirmation dialog for finishing an empty order
    def show_finish_empty_order_confirmation(self):

        # Ensure no other confirm frame is active
        self.remove_confirmation_buttons()
        if not self.root.winfo_exists(): 
            return

        self.confirm_frame = tk.Frame(self.root, bg=self.colors['background'])
        self.confirm_frame.pack(pady=10)
        
        tk.Label(self.confirm_frame, text="Finish with an empty order?", bg=self.colors['background'], fg=self.colors['text']).pack(pady=5)
        
        yes_btn = self.create_button(self.confirm_frame, text="Yes, Finish", command=lambda: self.handle_session_end(empty_order_confirmed=True), bg_color=self.colors['danger'], fg_color='white')
        yes_btn.pack(side=tk.LEFT, padx=10)
        
        no_btn = self.create_button(self.confirm_frame, text="No, Continue", command=self.continue_ordering_after_empty_finish_prompt, bg_color=self.colors['success'], fg_color='white')
        no_btn.pack(side=tk.LEFT, padx=10)


    # Handles "No" for finishing an empty order, goes back to ordering
    def continue_ordering_after_empty_finish_prompt(self):

        self.remove_confirmation_buttons() 
        self.continue_ordering()


    # Saves the final order to a text file
    def save_order_to_file(self):

        # Create timestamp on the order for easier management of orders
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')  
        filename = f"orders/Order_Table{self.selected_table}_{timestamp}.txt"        
        
        with open(filename, 'w') as f:

            f.write(f"Table: {self.selected_table}\n")
            f.write(f"Timestamp: {datetime.datetime.now().strftime('%Y-%m-%d %I:%M:%S %p')}\n")
            f.write("="*30 + "\n")
            f.write("Final Order Details:\n")
            
            # Count items from the flat self.all_orders list
            order_counts = defaultdict(int)
            for item in self.all_orders: 
                order_counts[item] += 1
            
            if order_counts:
                for item, count in order_counts.items():
                    # Format as "Item (xN)"
                    f.write(f"- {item.capitalize()} (x{count})\n") 

            else: # Shouldn't happen, just in case
                f.write("- No items ordered.\n")
            
            f.write("="*30 + "\n")
            # Total count of all individual items
            f.write(f"Total individual items: {len(self.all_orders)}\n")
        
        self.update_chat(f"System: Final order saved to {filename}")


    # Handles the end of the ordering session
    def handle_session_end(self, empty_order_confirmed=False):

        self.remove_confirmation_buttons() 
        self.remove_management_buttons() 
        
        # Create a file to signal to the movement section that the order has finished and the robot can return to the waiter
        if self.all_orders or empty_order_confirmed:
            with open("ReturnToWaiter", "w") as f: 
                pass 

        end_message = f"Thank you table {self.selected_table}! Your order is complete. Goodbye!"
        # If finished with an empty order
        if not self.all_orders and empty_order_confirmed:
            end_message = f"No order placed for table {self.selected_table}. Session ended. Goodbye!"
        # Shouldn't be reached, but may happen if orders folder doesn't exist. Doesn't close the program
        elif not self.all_orders and not empty_order_confirmed: 
            return 
        
        self.speak(end_message)
        self.update_chat(f"Waiter: {end_message}")
        
        # Disable all inputs as the application is closing
        self.disable_input() 
        if hasattr(self, 'record_btn') and self.record_btn.winfo_exists(): 
            self.record_btn.config(state=tk.DISABLED)
        if hasattr(self, 'user_input') and self.user_input.winfo_exists(): 
            self.user_input.config(state=tk.DISABLED)
        if hasattr(self, 'send_btn') and self.send_btn.winfo_exists(): 
            self.send_btn.config(state=tk.DISABLED)
        
        # Set shutdown flag
        self.shutting_down = True
        # Schedule window closure
        if self.root.winfo_exists():
            # Call on_close after 3 seconds for messages to be heard/seen
            self.root.after(3000, self.on_close) 


if __name__ == "__main__":

    # Create a default config.txt if it doesn't exist
    if not os.path.exists("config.txt"):
        with open("config.txt", "w") as f:
            f.write("[Number of Tables]\ncount = 10\n\n[Menu]\nitems = pizza, chicken, chips, coke, burger, salad, ice cream\n")
    
    # Create the main Tkinter window
    root = tk.Tk()
    # Instantiate the application class
    app = RobotWaiter(root)
    # Start program
    root.mainloop()  
