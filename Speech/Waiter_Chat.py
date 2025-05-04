import datetime
import tkinter as tk
from tkinter import scrolledtext, messagebox, simpledialog
import threading
import whisper
import numpy as np
import pyttsx3
import pyaudio
import os
import configparser
import string
import re
from collections import defaultdict

class RobotWaiter:

    def __init__(self, root):
        self.root = root
        self.root.title("Robot Waiter")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.engine_lock = threading.Lock()  # added to fix error
        
        # Colour scheme
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
        
        # Configure root window
        self.root.configure(bg=self.colors['background'])
        self.root.attributes('-fullscreen', True)
        
        # State management
        self.is_speaking = False
        self.shutting_down = False
        self.all_orders = []
        
        # Initialize components
        self.audio = pyaudio.PyAudio()
        self.model = whisper.load_model("base")
        
        # Read configuration
        self.config = self.read_config()
        self.table_count = self.config.getint('Number of Tables', 'count', fallback=10)
        self.table_count = max(1, min(40, self.table_count))
        self.menu = [item.strip() for item in 
                    self.config.get('Menu', 'items', fallback='pizza,chicken,fries,coke').split(',')]
        
        # Response settings
        self.yes_similes = ['yes', 'yeah', 'yep', 'sure', 'correct', 'right', 'affirmative', 'ok', 'okay']
        self.no_similes = ['no', 'nope', 'nah', 'no thank you', 'done', 'finished', 'negative', 'incorrect', 'wrong']
        self.current_order = []
        self.order_counter = 0
        self.is_recording = False
        self.selected_table = None
        self.engine = None
        
        # Show table selection first
        self.show_table_selection()
        
        # Escape key to exit fullscreen
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
    

    """Handle window close gracefully"""    
    def on_close(self):
        
        self.shutting_down = True
        if hasattr(self, 'engine') and self.engine:
            self.engine.stop()

        self.root.destroy()
    

    """Remove all punctuation from text"""
    def remove_punctuation(self, text):
        
        translator = str.maketrans('', '', string.punctuation)
        text = text.translate(translator)
        return re.sub(' +', ' ', text).strip().lower()
    

    """Check if text matches 'yes' responses"""
    def is_affirmative(self, text):

        clean_text = self.remove_punctuation(text)
        return any(word in clean_text for word in self.yes_similes)
    
    
    """Check if text matches 'no' responses"""
    def is_negative(self, text):

        clean_text = self.remove_punctuation(text)
        return any(word in clean_text for word in self.no_similes)
    

    """Disable all user input""" # For when the program waits to prevent crashing when multiple buttons are pressed
    def disable_input(self):

        self.is_speaking = True
        self.root.config(cursor="watch")
        self.root.bind("<Button-1>", lambda e: "break")
        self.root.bind("<Key>", lambda e: "break")

        if hasattr(self, 'record_btn'):
            self.record_btn.config(state=tk.DISABLED)
        if hasattr(self, 'user_input'):
            self.user_input.config(state=tk.DISABLED)
    
    
    """Re-enable user input"""
    def enable_input(self):

        self.is_speaking = False
        self.root.config(cursor="")
        self.root.unbind("<Button-1>")
        self.root.unbind("<Key>")

        if hasattr(self, 'record_btn'):
            self.record_btn.config(state=tk.NORMAL)
        if hasattr(self, 'user_input'):
            self.user_input.config(state=tk.NORMAL)
    
    
    """Create rectangle with rounded corners"""
    def create_rounded_rect(self, canvas, x1, y1, x2, y2, radius=25, **kwargs):
        
        points = [x1+radius, y1,
                 x1+radius, y1,
                 x2-radius, y1,
                 x2-radius, y1,
                 x2, y1,
                 x2, y1+radius,
                 x2, y1+radius,
                 x2, y2-radius,
                 x2, y2-radius,
                 x2, y2,
                 x2-radius, y2,
                 x2-radius, y2,
                 x1+radius, y2,
                 x1+radius, y2,
                 x1, y2,
                 x1, y2-radius,
                 x1, y2-radius,
                 x1, y1+radius,
                 x1, y1+radius,
                 x1, y1]
        return canvas.create_polygon(points, **kwargs, smooth=True)
    
    
    """Create a button"""
    def create_modern_button(self, parent, text, command, bg_color, fg_color='white', **kwargs):
        
        btn = tk.Button(

            parent,
            text=text,
            command=command,
            bg=bg_color,
            fg=fg_color,
            activebackground=self.adjust_color(bg_color, -20),
            activeforeground=fg_color,
            borderwidth=0,
            highlightthickness=0,
            relief='flat',
            font=('Arial', 12, 'bold'),
            padx=15,
            pady=8,
            **kwargs
        )
        btn.bind("<Enter>", lambda e: btn.config(bg=self.adjust_color(bg_color, -10)))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg_color))
        return btn
    
    
    """Lighten or darken a color"""
    def adjust_color(self, color, amount):
        
        from colorsys import rgb_to_hls, hls_to_rgb
        import re
        
        # Convert hex to rgb
        color = color.strip('#')
        rgb = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
        
        # Convert to HLS
        h, l, s = rgb_to_hls(*[x/255.0 for x in rgb])
        
        # Adjust lightness
        l = max(0, min(1, l + amount/100.0))
        
        # Convert back to RGB
        rgb = tuple(int(x*255) for x in hls_to_rgb(h, l, s))
        
        # Convert to hex
        return '#%02x%02x%02x' % rgb
    
    
    """Speak text using TTS with input blocking"""
    def speak(self, text):
        """Speak text using TTS with input blocking"""
        if self.shutting_down:
            return

        self.disable_input()

        def _speak():
            try:
                if hasattr(self, 'engine') and self.engine:
                    with self.engine_lock:  # fixes multiple runAndWait error
                        self.engine.say(text)
                        self.engine.runAndWait()
            finally:
                if not self.shutting_down:
                    self.root.after(0, self.enable_input)

        threading.Thread(target=_speak, daemon=True).start()
    
    
    """Read configuration from config.txt"""
    def read_config(self):
        
        config = configparser.ConfigParser()
        if not os.path.exists("config.txt"):

            with open("config.txt", "w") as f:
                f.write("[Number of Tables]\ncount = 10\n\n[Menu]\nitems = pizza, chicken, fries, coke\n")
        
        try:

            config.read("config.txt")
            if 'Number of Tables' not in config:
                config['Number of Tables'] = {'count': '10'}
            if 'Menu' not in config:
                config['Menu'] = {'items': 'pizza, chicken, fries, coke'}

            return config
        
        except Exception as e:

            messagebox.showwarning("Config Error", f"Error reading config.txt\nUsing default values\nError: {str(e)}")
            config['Number of Tables'] = {'count': '10'}
            config['Menu'] = {'items': 'pizza, chicken, fries, coke'}
            return config
    
    
    """Initialize TTS engine after table selection"""
    def initialize_tts(self):
        
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 180)
    
    
    """Show table selection buttons"""
    def show_table_selection(self):
        
        self.clear_gui()
        
        # Main container with padding
        container = tk.Frame(self.root, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=50, pady=50)
        
        # Header
        header = tk.Frame(container, bg=self.colors['background'])
        header.pack(fill=tk.X, pady=(0, 30))
        
        tk.Label(

            header,
            text="Please select your table number:",
            font=('Arial', 18, 'bold'),
            fg=self.colors['text'],
            bg=self.colors['background']
        ).pack()
        
        # Table buttons grid
        table_frame = tk.Frame(container, bg=self.colors['background'])
        table_frame.pack(expand=True)
        
        # Calculate rows and columns based on table count
        cols = min(10, self.table_count)
        rows = (self.table_count + cols - 1) // cols
        
        for i in range(1, self.table_count + 1):
            btn = self.create_modern_button(

                table_frame,
                text=str(i),
                command=lambda num=i: self.set_table(num),
                bg_color=self.colors['primary'],
                fg_color='white'
            )
            btn.grid(

                row=(i-1)//cols,
                column=(i-1)%cols,
                padx=10,
                pady=10,
                ipadx=20,
                ipady=15
            )
        
        # Footer with exit button
        footer = tk.Frame(container, bg=self.colors['background'])
        footer.pack(fill=tk.X, pady=(30, 0))
        
        self.create_modern_button(

            footer,
            text="Exit",
            command=self.on_close,
            bg_color=self.colors['danger'],
            fg_color='white'
        ).pack(side=tk.RIGHT)
    
    
    """Set the selected table number"""
    def set_table(self, table_num):
        
        self.selected_table = table_num
        self.initialize_tts()
        self.speak(f"Table {table_num} selected")
        self.clear_gui()
        self.create_main_interface()
        self.speak_welcome()
    
    
    """Clear all widgets from root"""
    def clear_gui(self):
        
        for widget in self.root.winfo_children():
            widget.destroy()
    
    
    """Create main ordering interface"""
    def create_main_interface(self):
        
        # Main container with padding
        container = tk.Frame(self.root, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=50, pady=50)
        
        # Header with table info
        header = tk.Frame(container, bg=self.colors['background'])
        header.pack(fill=tk.X, pady=(0, 20))
        
        tk.Label(

            header,
            text=f"Table: {self.selected_table}",
            font=('Arial', 16, 'bold'),
            fg=self.colors['primary'],
            bg=self.colors['background']
        ).pack(side=tk.LEFT)
        
        # Chat display area
        chat_frame = tk.Frame(container, bg=self.colors['background'])
        chat_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 20))
        
        # Chat display with border and padding
        self.chat_display = scrolledtext.ScrolledText(

            chat_frame,
            wrap=tk.WORD,
            font=('Arial', 12),
            bg=self.colors['card'],
            fg=self.colors['text'],
            bd=2,
            relief=tk.GROOVE,
            highlightthickness=0,
            padx=20,
            pady=20
        )

        self.chat_display.pack(fill=tk.BOTH, expand=True)
        self.chat_display.tag_config('user', foreground=self.colors['primary'])
        self.chat_display.tag_config('bot', foreground=self.colors['secondary'])
        self.chat_display.config(state=tk.DISABLED)
        
        # Input area
        input_frame = tk.Frame(container, bg=self.colors['background'])
        input_frame.pack(fill=tk.X, pady=(0, 20))
        
        self.user_input = tk.Entry(

            input_frame,
            font=('Arial', 12),
            bg=self.colors['card'],
            fg=self.colors['text'],
            bd=2,
            relief=tk.GROOVE,
            highlightthickness=0,
            insertbackground=self.colors['text']
        )
        self.user_input.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10), pady=10)
        self.user_input.bind("<Return>", lambda e: self.process_text_input())
        
        send_btn = self.create_modern_button(

            input_frame,
            text="Send",
            command=self.process_text_input,
            bg_color=self.colors['accent'],
            fg_color='white'
        )
        send_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Button area
        button_frame = tk.Frame(container, bg=self.colors['background'])
        button_frame.pack(fill=tk.X)
        
        self.record_btn = self.create_modern_button(

            button_frame,
            text="🎤 Record Order",
            command=self.start_recording,
            bg_color=self.colors['primary'],
            fg_color='white'
        )
        self.record_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Status bar
        self.status = tk.Label(

            container,
            text="Ready",
            bd=0,
            relief=tk.FLAT,
            anchor=tk.W,
            font=('Arial', 10),
            fg=self.colors['text'],
            bg=self.colors['background']
        )
        self.status.pack(fill=tk.X, pady=(10, 0))
    
    
    """Initial welcome message"""
    def speak_welcome(self):
        
        welcome_msg = f"Welcome table {self.selected_table}! Our menu has: " + ", ".join(self.menu)
        self.speak(welcome_msg)
        self.update_chat(f"Waiter: {welcome_msg}")
    
    
    """Start recording using Main.py approach"""
    def start_recording(self):
        
        if self.is_recording:
            return
            
        self.is_recording = True
        self.record_btn.config(state=tk.DISABLED)
        self.set_status("Preparing to record...")
        threading.Thread(target=self.record_and_process, daemon=True).start()
    
    
    """Record and process audio like Main.py"""
    def record_and_process(self):
        
        try:
            self.set_status("Recording... Speak now!")
            self.speak("Please place your order now")
            
            FORMAT = pyaudio.paInt16
            CHANNELS = 1
            RATE = 16000
            CHUNK = 1024
            RECORD_SECONDS = 5
            
            stream = self.audio.open(

                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK
            )
            
            frames = []
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                if not self.is_recording:
                    break
                data = stream.read(CHUNK)
                frames.append(data)
            
            stream.stop_stream()
            stream.close()
            
            if frames:
                audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
                self.process_audio_data(audio_data)
            
        except Exception as e:
            self.set_status(f"Error: {str(e)}")
            self.speak("Sorry, there was an error with the recording")
            self.update_chat(f"System Error: {str(e)}")
        finally:
            self.is_recording = False
            self.root.after(0, lambda: self.record_btn.config(state=tk.NORMAL))
            self.set_status("Ready")
    
    
    """Process audio data with punctuation removal"""
    def process_audio_data(self, audio_data):
        
        try:
            audio_np = audio_data.astype(np.float32) / 32768.0
            result = self.model.transcribe(audio_np, language='en')
            order_text = self.remove_punctuation(result['text'].strip())
            self.update_chat(f"You (audio): {order_text}", is_user=True)
            self.process_order_text(order_text)
        except Exception as e:
            self.speak("Sorry, I couldn't process that order")
            self.update_chat(f"System Error: {str(e)}")
    
    
    """Handle text input submission"""
    def process_text_input(self):
       
        text = self.user_input.get().strip()
        if not text:
            return
            
        self.update_chat(f"You: {text}", is_user=True)
        self.user_input.delete(0, tk.END)
        self.process_order_text(text)
    
    
    """Process order text with punctuation removed"""
    def process_order_text(self, text):
        
        clean_text = self.remove_punctuation(text)
        
        # Check if in confirmation mode and handle yes/no responses
        if hasattr(self, 'confirm_frame') and self.confirm_frame.winfo_ismapped():

            if self.is_affirmative(clean_text):
                self.confirm_order_yes()
                return
            elif self.is_negative(clean_text):
                self.confirm_order_no()
                return
        
        # Original order processing logic
        items = [word for word in clean_text.split() if word in self.menu]
        
        if items:
            self.current_order = items
            response = f"I heard: {', '.join(items)}. Is this correct?"
            self.speak(response)
            self.update_chat(f"Waiter: {response}")
            self.show_confirmation_buttons()
        else:
            response = "I didn't recognize any menu items. Our menu has: " + ", ".join(self.menu)
            self.speak(response)
            self.update_chat(f"Waiter: {response}")
    
    
    """Add message to chat display"""
    def update_chat(self, message, is_user=False):
        
        self.chat_display.config(state=tk.NORMAL)
        tag = "user" if is_user else "bot"
        self.chat_display.insert(tk.END, f"{message}\n", tag)
        self.chat_display.config(state=tk.DISABLED)
        self.chat_display.see(tk.END)
    
    
    """Update status bar"""
    def set_status(self, message):
        
        self.status.config(text=message)
    
    
    """Show Yes/No confirmation buttons"""
    def show_confirmation_buttons(self):
        
        self.remove_confirmation_buttons()
        
        self.confirm_frame = tk.Frame(self.root, bg=self.colors['background'])
        self.confirm_frame.pack(pady=10)
        
        self.create_modern_button(

            self.confirm_frame,
            text="Yes",
            command=self.confirm_order_yes,
            bg_color=self.colors['success'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=10)
        
        self.create_modern_button(

            self.confirm_frame,
            text="No",
            command=self.confirm_order_no,
            bg_color=self.colors['danger'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=10)
    
    
    """Remove confirmation buttons if they exist"""
    def remove_confirmation_buttons(self):
        
        if hasattr(self, 'confirm_frame') and self.confirm_frame:

            self.confirm_frame.pack_forget()
            self.confirm_frame.destroy()
            del self.confirm_frame
    
    
    """Handle order confirmation"""
    def confirm_order_yes(self):
        
        self.update_chat("You: Yes", is_user=True)
        if self.current_order:

            self.all_orders.extend(self.current_order)
            response = f"Order confirmed! You ordered: {', '.join(self.current_order)}"
            self.speak(response)
            self.update_chat(f"Waiter: {response}")
            self.show_order_management()
        
        self.remove_confirmation_buttons()
        self.current_order = []
    
    
    """Handle order rejection"""
    def confirm_order_no(self):
        
        self.update_chat("You: No", is_user=True)
        self.remove_confirmation_buttons()
        response = "Let's try again. What would you like to order?"
        self.speak(response)
        self.update_chat(f"Waiter: {response}")
        self.current_order = []
    
    
    """Show order management options"""
    def show_order_management(self):
        
        self.remove_confirmation_buttons()
        
        self.management_frame = tk.Frame(self.root, bg=self.colors['background'])
        self.management_frame.pack(pady=10)
        
        self.create_modern_button(

            self.management_frame,
            text="View/Edit Orders",
            command=self.show_full_order,
            bg_color=self.colors['warning'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=10)
        
        self.create_modern_button(

            self.management_frame,
            text="Continue Ordering",
            command=self.continue_ordering,
            bg_color=self.colors['success'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=10)
        
        self.create_modern_button(

            self.management_frame,
            text="Finish & Pay",
            command=self.finish_order,
            bg_color=self.colors['danger'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=10)
    
    """Show current order with edit options dialogue box"""
    def show_full_order(self):
        
        if not self.all_orders:

            self.speak("You haven't ordered anything yet")
            return
            
        order_window = tk.Toplevel(self.root)
        order_window.title("Your Current Order")
        order_window.configure(bg=self.colors['background'])
        
        # Center the window
        window_width = 500
        window_height = 400
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        order_window.geometry(f"{window_width}x{window_height}+{x}+{y}")
        
        # Main container
        container = tk.Frame(order_window, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
        
        # Header
        tk.Label(

            container,
            text="Your Current Order",
            font=('Arial', 16, 'bold'),
            fg=self.colors['text'],
            bg=self.colors['background']
        ).pack(pady=(0, 15))
        
        # Order list in a simple frame with scrollbar
        list_frame = tk.Frame(container, bg=self.colors['background'])
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.order_listbox = tk.Listbox(

            list_frame,
            yscrollcommand=scrollbar.set,
            bg=self.colors['card'],
            fg=self.colors['text'],
            bd=2,
            relief=tk.GROOVE,
            highlightthickness=0,
            font=('Arial', 12),
            selectbackground=self.colors['accent']
        )

        self.order_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.order_listbox.yview)
        
        for idx, item in enumerate(self.all_orders, 1):
            self.order_listbox.insert(tk.END, f"{idx}. {item}")
        
        # Button frame
        btn_frame = tk.Frame(container, bg=self.colors['background'])
        btn_frame.pack(fill=tk.X, pady=(15, 0))
        
        self.create_modern_button(

            btn_frame,
            text="Remove Selected",
            command=self.remove_selected_item,
            bg_color=self.colors['danger'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=5)
        
        self.create_modern_button(

            btn_frame,
            text="Swap Selected",
            command=self.swap_selected_item,
            bg_color=self.colors['primary'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=5)
        
        self.create_modern_button(

            btn_frame,
            text="Close",
            command=order_window.destroy,
            bg_color=self.colors['secondary'],
            fg_color='white'
        ).pack(side=tk.RIGHT, padx=5)
    
    
    """Remove selected item from order"""
    def remove_selected_item(self):
        
        selection = self.order_listbox.curselection()
        if not selection:
            return
            
        index = selection[0]
        removed_item = self.all_orders.pop(index)
        self.order_listbox.delete(index)
        self.speak(f"Removed {removed_item}")
        
        self.order_listbox.delete(0, tk.END)
        for idx, item in enumerate(self.all_orders, 1):
            self.order_listbox.insert(tk.END, f"{idx}. {item}")
    
    
    """Swap selected item with a different menu item"""
    def swap_selected_item(self):
        
        selection = self.order_listbox.curselection()
        if not selection:
            return
            
        index = selection[0]
        current_item = self.all_orders[index]
        
        swap_window = tk.Toplevel(self.root)
        swap_window.title("Swap Item")
        swap_window.configure(bg=self.colors['background'])
        
        # Center the window
        window_width = 300
        window_height = 300
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        swap_window.geometry(f"{window_width}x{window_height}+{x}+{y}")
        
        # Main container
        container = tk.Frame(swap_window, bg=self.colors['background'])
        container.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
        
        tk.Label(

            container,
            text=f"Swap {current_item} with:",
            font=('Arial', 14),
            fg=self.colors['text'],
            bg=self.colors['background']
        ).pack(pady=(0, 15))
        
        # Menu list in a frame
        list_frame = tk.Frame(container, bg=self.colors['background'])
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        menu_listbox = tk.Listbox(

            list_frame,
            bg=self.colors['card'],
            fg=self.colors['text'],
            bd=2,
            relief=tk.GROOVE,
            highlightthickness=0,
            font=('Arial', 12),
            selectbackground=self.colors['accent']
        )
        menu_listbox.pack(fill=tk.BOTH, expand=True)
        
        for item in self.menu:
            menu_listbox.insert(tk.END, item)
        
        # Swap button
        btn_frame = tk.Frame(container, bg=self.colors['background'])
        btn_frame.pack(fill=tk.X, pady=(15, 0))
        
        self.create_modern_button(

            btn_frame,
            text="Swap",
            command=lambda: self.perform_swap(swap_window, menu_listbox, index, current_item),
            bg_color=self.colors['success'],
            fg_color='white'
        ).pack(side=tk.LEFT, padx=5)
        
        self.create_modern_button(

            btn_frame,
            text="Cancel",
            command=swap_window.destroy,
            bg_color=self.colors['danger'],
            fg_color='white'
        ).pack(side=tk.RIGHT, padx=5)
    
    
    """Perform the item swap"""
    def perform_swap(self, window, listbox, index, current_item):
        
        swap_selection = listbox.curselection()
        if not swap_selection:
            return
            
        new_item = self.menu[swap_selection[0]]
        self.all_orders[index] = new_item
        self.speak(f"Swapped {current_item} with {new_item}")
        
        self.order_listbox.delete(index)
        self.order_listbox.insert(index, f"{index+1}. {new_item}")
        window.destroy()
    
    
    """Continue with additional orders"""
    def continue_ordering(self):
        
        if hasattr(self, 'management_frame'):
            self.management_frame.pack_forget()
            self.management_frame.destroy()

        self.speak("What would you like to order next?")
    
    
    """Finalize order and close"""
    def finish_order(self):
        
        if hasattr(self, 'management_frame'):
            self.management_frame.pack_forget()
            self.management_frame.destroy()
        
        if self.all_orders:
            self.save_order_to_file()
            self.handle_session_end()
        else:
            self.speak("You haven't ordered anything yet")
            self.update_chat("Waiter: You haven't ordered anything yet")
    
    
    """Save complete order to text file"""
    def save_order_to_file(self):
        
        timestamp = datetime.datetime.now().strftime('%I.%M_%p_%S')
        filename = f"orders/Order_{self.selected_table}_{timestamp}.txt"        
        with open(filename, 'w') as f:

            f.write(f"Table: {self.selected_table}\n")
            f.write("Final Order Details:\n")
            f.write("="*20 + "\n")
            
            order_counts = defaultdict(int)
            for item in self.all_orders:
                order_counts[item] += 1
            
            for item, count in order_counts.items():
                if count > 1:
                    f.write(f"- {item} (x{count})\n")
                else:
                    f.write(f"- {item}\n")
            
            f.write("\nTotal items: " + str(len(self.all_orders)))
        
        self.update_chat(f"System: Final order saved to {filename}")
    
    
    """Create empty file and close GUI"""
    def handle_session_end(self):
        
        with open("ReturnToWaiter", "w") as f:
            pass
        
        self.speak(f"Thank you table {self.selected_table}! Goodbye!")
        self.update_chat(f"Waiter: Thank you table {self.selected_table}! Goodbye!")
        self.root.after(2000, self.root.destroy)


if __name__ == "__main__":

    if not os.path.exists("config.txt"):

        with open("config.txt", "w") as f:

            f.write("[Number of Tables]\ncount = 10\n\n[Menu]\nitems = pizza, chicken, fries, coke\n")
    
    root = tk.Tk()
    app = RobotWaiter(root)
    root.mainloop()