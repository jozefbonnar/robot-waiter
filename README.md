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

The Navigation system is responsible for:
- Navigating to the customer to take an order
- Avoiding obstacles along the way using sensor data
- Returning to its initial position after collecting the order, while continuing to avoid obstacles

## How to Use Navigation Module:
#### Robot Waiter Autonomous Movement

This handles navigation to the customer and back to the starting point, with real-time obstacle avoidance.

## Prerequisites:

- Docker installed
- Ubuntu operating system(
- Docker image with ROS Noetic and necessary dependencies

#### Clone the Project Repository:
~~~
git clone https://github.com/jozefbonnar/robot-waiter.git
~~~
#### file structure :
~~~
project/

├── LCASTOR

├── robot-waiter
~~~
#### Mount the Directory to Your Docker Container:

- If you have an **NVIDIA GPU**, use **run_docker.sh** to launch the container with GPU support.
- If you do not have an **NVIDIA GPU**, use **run_cpu_docker.sh** for the CPU-based container.

~~~
project/

├── LCASTOR
      └── lcastor_docker
              └── run_docker.sh(NVIDIA GPU required) or run_cpu_docker.sh(CPU based)

├── robot-waiter 
~~~
- In the chosen script, add the following volume mount line.This line ensures that the **tse_local_dir** directory in the robot-waiter project is correctly mounted to the Docker container.
~~~
-v $(pwd)/../../robot-waiter/Navigation/tse_local_dir:/home/lcastor/ros_ws/src/tse_local_dir \
~~~
#### Run the image(container):
For NVIDIA GPU:
~~~
./run_docker.sh
~~~
For CPU (no GPU):
~~~
./run_cpu_docker.sh
~~~
#### Launch the **Navigation Stack** via tmule:
~~~
tmule -c $(rospack find tse_navigation)/tmule/tse_simulated.yaml -W 3 launch
~~~
#### Send a Goal:

- Either through RViz or a custom interface, send the robot to Unknown Environment.

#### Run the Navigation Node:
~~~
rosrun tse_navigation nav_and_return
~~~

#### Robot Behavior:

- The robot will navigate to the customer using real-time SLAM.
- It will avoid all detected obstacles on the way.
- Once the order is taken, it automatically returns to its starting location, again avoiding obstacles.

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
