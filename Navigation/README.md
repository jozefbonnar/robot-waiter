## Navigation 
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
