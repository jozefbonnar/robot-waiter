# Robot Waiter

A software solution for an autonomous robot waiter designed to serve customers in bars and restaurants.

## Project Overview

This project implements a robot waiter capable of:
1. Identifying customers who need service
2. Navigating to their table
3. Taking orders through speech recognition
4. Returning to the bar/kitchen to relay orders
5. Serving customers efficiently

The system was developed for simulation environments with the capability to be deployed on real robot hardware.

# Key Components

## Navigation

#TODO

## Speech Recognition

The speech recognition system is responsible for:
- Listening to customer input
- Taking down orders
- Confirming orders
- Relaying the information to the waiter


## How to use Speech Module:

#### Robot Waiter Customer Interaction

This is the interface with GUI and voice control for ordering items off the menu.

##### How to Run

###### Prerequisites

- Python 3.10.9 (Specifically for the combination of dependencies used)

- Microphone access (Speech to text)
- Speaker access (Text to speech)
  

###### Installation

1. **Load the Project in a .venv python environment (Python 3.10.9)**

2. **Install OpenAI-Whisper:**

```

pip install git+https://github.com/openai/whisper.git

```

3. **Install Dependencies**:

```

pip install -r requirements.txt

```

  

###### File Structure

```

project/

├── Waiter_Chat.py

├── Config.txt

├── requirements.txt

├── ReturnToWaiter (Created after an order)

└── Orders/

    └── (Orders will appear here once created)

```

  

###### Running the Program

1. **Start the python file:**

```

python Waiter_Chat.py

```

  

2. **Interact with the GUI**
   The GUI should launch into Fullscreen mode, If this has not happened then there is an error

3. **Select table number**

4. **Type your order into the text box or speak after the microphone button**

5. **Continue ordering more** or **Select, Enter or Speak YES or NO**
   Orders can also be edited using the edit button


###### After Running

1. The Program will close Once the Pay/Exit button is selected
2. Once the program closes two files are created:
   - Order_(TableNo)\_(Date&Time).txt (File containing order details for the human waiter)
   - ReturnToWaiter (File created to prompt the Movement part of the robot to Return to the bar/human waiter)



## Computer Vision

The vision system is responsible for:
- Detecting customers who are waving for service
- Detecting the distance from the customer


# Development and Testing

The solution was  developed in simulation environments. This approach allowed for rapid iteration and testing of algorithms before deployment in real-world settings.

# Team

This project was developed as part of CMP2804 - Team Software Engineering at the University of Lincoln.
