# Robot Waiter Vision Module

A robust machine vision system for detecting customers waving for service and estimating their distance, designed for integration with the Robot Waiter project.

## Overview

This module uses an iPhone's RGB-D camera (TrueDepth or LiDAR) to:
- Detect when a person is waving (hand raised above chest for a configurable duration)
- Measure the distance to the detected person using real depth data
- Provide real-time visual feedback for debugging and transparency

It is implemented in Python using OpenCV, MediaPipe, NumPy, and Record3D.

## Features

- **Gesture Recognition:** Detects hand-raising gestures using MediaPipe Pose. Requires the gesture to be held for a set duration (default: 5 seconds) to confirm intent and reduce false positives.
- **Depth Measurement:** Uses iPhone RGB-D data to calculate the distance to the customer, filtering out invalid values and using the 25th percentile of valid depths for robustness.
- **Person Segmentation:** Utilizes MediaPipe Selfie Segmentation to create an accurate person mask, improving both gesture detection and distance estimation.
- **Sensor Adaptation:** Handles both TrueDepth and LiDAR iPhones, including mirroring and confidence map display.
- **Visual Debugging:** Displays overlays for pose, segmentation mask, depth map, and confidence map. User can toggle debug and confidence displays, and adjust gesture confirmation time via keyboard shortcuts.
- **Modular Design:** The system is encapsulated in a single class (`RobotWaiterVision`) for easy integration and extension.

## System Architecture

- **Data Acquisition:** Streams RGB, depth, and confidence data from an iPhone using Record3D.
- **Preprocessing:** Rotates all images 90° to standardize orientation. Handles mirroring for TrueDepth sensors.
- **Pose & Segmentation:** Runs MediaPipe Pose and Selfie Segmentation on the rotated RGB image.
- **Gesture Detection:** Checks if either hand is above the chest and tracks the duration. Only triggers detection if the gesture is held for the required time.
- **Distance Calculation:** Applies the person mask to the depth map, filters outliers, and computes the 25th percentile distance.
- **Visualization:** Overlays pose, mask, and distance info on the RGB image. Shows depth and confidence maps in separate windows.

## Usage

1. **Install dependencies:**
   - Python 3.8+
   - OpenCV (`opencv-python`)
   - NumPy
   - MediaPipe
   - Record3D (and Record3D iOS app)

2. **Connect your iPhone** (with TrueDepth or LiDAR) and start the Record3D app.

3. **Run the module:**
   ```bash
   python robot_waiter_vision.py
   ```

4. **Controls:**
   - `ESC`: Exit
   - `d`: Toggle debug mode
   - `c`: Toggle confidence map display
   - `t`: Cycle required hand-raise duration (1, 3, 5, 10 seconds)

## Dependencies
- Python 3.8+
- opencv-python
- numpy
- mediapipe
- record3d

## Testing & Evaluation
- The system was tested with both TrueDepth and LiDAR iPhones in various lighting and background conditions.
- Robustness was evaluated by varying gesture duration, distance, and background clutter.
- The use of segmentation and statistical depth filtering significantly reduced false positives and improved distance accuracy.

## Example Output
- Main window: RGB image with pose and status overlay
- Additional windows: Depth map, confidence map, person mask, masked depth (in debug mode)

## Further Development
- Integrate with robot navigation and speech modules
- Support for multiple people and prioritization
- Improved gesture vocabulary (e.g., waving motion detection)

---
For implementation details, see `robot_waiter_vision.py`.
