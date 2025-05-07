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


## Speech Recognition

The speech recognition system is responsible for:
- Listening to customer input
- Taking down orders
- Confirming orders
- Relaying the information to the waiter

## Computer Vision

The vision system enables the robot to detect and respond to customers who are waving for service, as well as to estimate their distance. It is designed for robustness and real-world deployment, leveraging advanced machine vision techniques and depth sensing from modern iPhones.

**Key Features:**
- **Gesture Recognition:** Detects when a customer raises a hand above chest level and confirms the gesture only if sustained for a configurable duration (default: 5 seconds), reducing false positives.
- **Depth Sensing:** Uses RGB-D data from an iPhone (TrueDepth or LiDAR) to measure the distance to the detected person, providing reliable, real-world distance estimates.
- **Person Segmentation:** Employs MediaPipe Selfie Segmentation to accurately isolate the customer from the background, improving both gesture detection and distance measurement.
- **Visual Feedback:** Displays real-time overlays, including pose landmarks, segmentation masks, depth maps, and confidence maps, to aid debugging and system transparency.
- **Sensor Adaptation:** Automatically handles differences between iPhone sensor types (TrueDepth/LiDAR), including image mirroring and confidence map display.
- **Robustness:** Filters out invalid depth values and uses statistical measures (25th percentile) to provide stable distance readings, even in noisy environments.
- **User Controls:** Allows toggling of debug and confidence map displays, and adjustment of gesture confirmation time, via keyboard shortcuts.

**Technologies Used:**
- Python, OpenCV, NumPy
- MediaPipe (Pose and Selfie Segmentation)
- Record3D (for iPhone RGB-D streaming)

This system is modular and well-documented, making it suitable for both research and commercial deployment. For more details, see the dedicated Vision module documentation in `Vision/README.md`.

# Development and Testing

The solution was  developed in simulation environments. This approach allowed for rapid iteration and testing of algorithms before deployment in real-world settings.

# Team

This project was developed as part of CMP2804 - Team Software Engineering at the University of Lincoln.
