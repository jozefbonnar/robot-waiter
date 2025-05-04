import string
import speech_recognition as sr
import whisper
import numpy as np
import pyttsx3
import re
class ReasoningNode:
    def __init__(self):
        self.menu = ['pizza', 'chicken', 'fries', 'coke'] # Current menu (could just pass menu instead to the class)
        self.yesSimiles = ['yes', 'yeah', 'yep', 'yessir', 'i guess', 'why not', 'can', 'sure', 'correct', '100' 'hundred', 'deffo', 'daffo'] # Yes similes
        self.noSimiles = ['no', 'nope', 'nada', 'nah', 'pass', 'incorrect', "fail", "failed", 'no way', 'no chance'] # No similes
        self.engine = pyttsx3.init() # Initializes pyttsx3 engine
        voices = self.engine.getProperty('voices') 
        self.voiceId = voices[1].id  # Selects second available voice
        self.engine.setProperty('voice', self.voiceId) # Uses set voice 
        self.engine.setProperty('rate', 220)  # Speed
        self.engine.setProperty('volume', 0.1)  # Volume
        self.model = whisper.load_model("base") # Whisper model
        # Initialize speech recognizer and mic
        self.robot = sr.Recognizer()
        self.mic = sr.Microphone(device_index=1, sample_rate=16000)
        self.orderList = [] # Empty order list

    def speakMenu(self): # Speak menu function
        self.engine.say(f"The menu contains {self.menu}")
        self.engine.runAndWait()
    
    
    def checkIfMenuQuestion(self, customerText):
        question = False  # To check later
        itemFound = False # To check later
        customerText = customerText.lower() # Lower for parsing
        if "is" in customerText or "do you have" in customerText: # If question
            question = True
    
        if question:
            # Check if it is a menu item
            for item in self.menu:
                if item.lower() in customerText:
                    itemFound = True  # An item was found 
                    
                    
                    if itemFound: # Is item was on menu
                        response = f"Yes, we have {item} on the menu."
                        self.engine.say(response)
                        self.engine.runAndWait()
                        print(response)
                        return True  # Item on menu

                
                if not itemFound: # If item was not on menu
                    customerText = re.sub(r'\bis\b', '', customerText).strip() # Strip redundant "is" using boundaries to ensure "is" is a standalone word
                    customerText = customerText.replace("do you have", "").strip()
                    response = f"Sorry, we don't have {customerText} on the menu."
                    self.engine.say(response)
                    self.engine.runAndWait()
                    #print(response)
                    return False  # Item not on menu
                
        
        return False # No question asked

    def listening(self):
        with self.mic as source:
            self.robot.adjust_for_ambient_noise(source, duration=1)  # Adjust ambient noise
            print("Listening...")
            customerAudio = self.robot.listen(source, timeout=15, phrase_time_limit=None)  # Timeout after 15 seconds of no input
            return customerAudio  # Returns raw audio data for order

    def processAudioData(self, audioData):
        orderData = np.frombuffer(audioData, dtype=np.int16) # Convert audio into numpy array
        print("Raw audio data " + str(orderData.shape))
        orderData = orderData.astype(np.float32) / 32768.0  # # Normalize audio data to the range [-1, 1] by dividing by 32768 for Whisper to work, Convert to float32
        return orderData

    def checkForDone(self, resultText):
        return "done" in resultText.lower() or "finished" in resultText.lower() # Check if "done" or "finished"

    def removePunctuation(self, text):
        removePunc = str.maketrans('', '', string.punctuation) 
        return text.translate(removePunc) # Remove punctuation 

    def parseOrderForMenuItems(self, orderText):
        
        noPuncOrderText = self.removePunctuation(orderText).lower() # Remove puncuation, convert to lowercase
      
        orderWords = noPuncOrderText.split()   # Split text into words
        orderedItems = []

        for item in self.menu: # Loop through menu, check if each item is in order
            if item.lower() in orderWords:
                orderedItems.append(item)
        return orderedItems

    def processOrder(self):
        try:
            order = ""
            while True:
                
                customerAudio = self.listening() # Capture customer audio 

                try:
                    orderData = self.processAudioData(customerAudio.get_raw_data()) # Process audio into correct format for Whisper
                    result = self.model.transcribe(orderData, language='en', task='transcribe')  # Use Whisper, audio into text, specify language and task
                    print("Customer input ", result['text'])
                    order += result['text'] + " "  # Append order, concatenate the order
                    print("Customer full input " + str(order))

                    if self.checkForDone(result['text']):  # If "done"
                        print("Order finished.")
                        break  # Break if "done"
                    
                    self.checkIfMenuQuestion(result['text'])

                except Exception as e:
                    print("Error during transcription " + str(e))

            orderedItems = self.parseOrderForMenuItems(order) # After finishing order, parse for menu items
            if orderedItems:
                readOrder = ("You ordered, " + str(orderedItems)) 
                self.engine.say(readOrder)
                self.engine.runAndWait()
                self.validateOrder(orderedItems)  # Validate order
            else:
                readFailedOrder = ("No valid menu items in given order.")
                self.engine.say(readFailedOrder)
                self.engine.runAndWait()

        except KeyboardInterrupt:
            print("Order received.")

    def validateOrder(self, orderedItems):
        self.engine.say("Is that correct?")
        self.engine.runAndWait()
        customerAudio = self.listening() # Listen for response
        try:
            orderData = self.processAudioData(customerAudio.get_raw_data()) # Process captured audio
            result = self.model.transcribe(orderData, language='en', task='transcribe')
            response = result['text'].lower()
            print("Customer response: ", response)

            if any(yesSimile in response for yesSimile in self.yesSimiles): # If yes
                self.engine.say("Order confirmed. Thank you!")
                self.engine.runAndWait()
                print("Order confirmed.")
                self.orderList.append(orderedItems)
                self.engine.say("Would you like to do another order?")
                self.engine.runAndWait()
                customerAudio = self.listening()
                orderData = self.processAudioData(customerAudio.get_raw_data())
                result = self.model.transcribe(orderData, language='en', task='transcribe')
                response = result['text'].lower()
                print("Customer response: ", response)

                
                if any(noSimile in response for noSimile in self.noSimiles): # Check if noSimiles
                    if len(self.orderList) > 1:
                        self.engine.say(f"Thank you for your orders of")
                        self.engine.runAndWait()
                        for item in self.orderList:
                            self.engine.say(f"{item}")
                            self.engine.runAndWait()
                            
                        return False
                    else:
                        self.engine.say(f"Thank you for your order of {self.orderList}")
                        self.engine.runAndWait()
                        return False

                else: # IF not no then yes
                    self.engine.say("Let's take another order.")
                    self.engine.runAndWait()
                    print("Proceeding with a futher order.")
                    self.processOrder()
                    
                return True
            elif any(noSimile in response for noSimile in self.noSimiles): # If wrong order
                self.engine.say("Please repeat your order.") # Prompt reorder
                self.engine.runAndWait()
                print("Order dismissed, Restarting...")
                self.processOrder()  # Restart the order process
            else:
                self.engine.say("Please say your answer again.") # No input given
                self.engine.runAndWait()
                self.validateOrder(orderedItems)  # Ask for validation again 
           
        except Exception as e:
            print("Error during validation " + str(e))


reasoningNode = ReasoningNode() # Create ReasoningNode class instance
reasoningNode.speakMenu() # Call speakMenu
reasoningNode.processOrder() # Start order process