import numpy as np
from record3d import Record3DStream
import cv2
import mediapipe as mp
from threading import Event
import math
import time


class RobotWaiterVision:
    def __init__(self):
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1
        
        #ediaPipe pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.8,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        
        #MediaPipe selfie segmentation for better person mask
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.selfie_segmentation = self.mp_selfie_segmentation.SelfieSegmentation(model_selection=1)
        
        #Debug mode - set to True to show additional information
        self.debug_mode = True
        
        #Always show confidence map (Optional)
        self.show_confidence = True
        
        #Hand raising tracking variables
        self.hand_raised_start_time = None
        self.required_raise_duration = 5.0  # seconds
        self.confirmed_hand_raised = False

    def on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  #Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  #Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])
    
    def rotate_image(self, image, rotation=90):
        """
        Rotate image clockwise by the specified angle
        """
        if rotation == 90:
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif rotation == 180:
            return cv2.rotate(image, cv2.ROTATE_180)
        elif rotation == 270:
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return image
    
    def detect_hand_raised(self, pose_landmarks, visualization=None):
        """
        Simple detection: check if either hand is above the chest level
        """
        if not pose_landmarks:
            return False
        
        #Get relevant landmarks
        right_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_wrist = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST]
        
        #Get shoulders
        right_shoulder = pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_shoulder = pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        
        #Get chest level (midpoint of shoulders)
        chest_y = (right_shoulder.y + left_shoulder.y) / 2
        
        #Check if either hand is raised above chest
        right_hand_raised = right_wrist.y < chest_y - 0.05  # Add a small margin
        left_hand_raised = left_wrist.y < chest_y - 0.05    # Add a small margin
        
        #Determine if any hand is raised
        current_hand_raised = right_hand_raised or left_hand_raised
        
        #Track time for continuous hand raising
        current_time = time.time()
        
        if current_hand_raised:
            #If hand just started being raised, record the start time
            if self.hand_raised_start_time is None:
                self.hand_raised_start_time = current_time
                self.confirmed_hand_raised = False
            
            #Check if hand has been raised for required duration
            elif current_time - self.hand_raised_start_time >= self.required_raise_duration:
                self.confirmed_hand_raised = True
        else:
            #Reset if hand is lowered
            self.hand_raised_start_time = None
            self.confirmed_hand_raised = False
        
        #Draw debug visualization
        if visualization is not None and self.debug_mode:
            h, w, _ = visualization.shape
            
            #Draw chest line
            chest_y_px = int(chest_y * h)
            cv2.line(visualization, (0, chest_y_px), (w, chest_y_px), (255, 0, 255), 2)
            cv2.putText(visualization, "CHEST LEVEL", (10, chest_y_px - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            #Mark hands with colored circles
            r_color = (0, 255, 0) if right_hand_raised else (0, 0, 255)
            l_color = (0, 255, 0) if left_hand_raised else (0, 0, 255)
            
            r_wrist_px = (int(right_wrist.x * w), int(right_wrist.y * h))
            l_wrist_px = (int(left_wrist.x * w), int(left_wrist.y * h))
            
            cv2.circle(visualization, r_wrist_px, 10, r_color, -1)
            cv2.circle(visualization, l_wrist_px, 10, l_color, -1)
            
            #Add hand status text
            r_status = "R: RAISED" if right_hand_raised else "R: DOWN"
            l_status = "L: RAISED" if left_hand_raised else "L: DOWN"
            
            cv2.putText(visualization, r_status, (w - 120, h - 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, r_color, 2)
            cv2.putText(visualization, l_status, (w - 120, h - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, l_color, 2)
            
            #Add timer information if hand is raised but not confirmed
            if current_hand_raised and not self.confirmed_hand_raised:
                elapsed_time = current_time - self.hand_raised_start_time
                remaining_time = max(0, self.required_raise_duration - elapsed_time)
                timer_text = f"Keep hand raised: {remaining_time:.1f}s"
                
                # Draw progress bar
                progress = min(1.0, elapsed_time / self.required_raise_duration)
                bar_width = int(w * 0.6)
                bar_height = 15
                bar_x = int((w - bar_width) / 2)
                bar_y = 120
                
                #Background bar
                cv2.rectangle(visualization, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                              (100, 100, 100), -1)
                
                #Progress bar
                progress_width = int(bar_width * progress)
                progress_color = (0, 255, 255)  # Yellow
                if progress > 0.7:
                    progress_color = (0, 255, 0)  # Green when almost done
                cv2.rectangle(visualization, (bar_x, bar_y), (bar_x + progress_width, bar_y + bar_height), 
                              progress_color, -1)
                
                #Border
                cv2.rectangle(visualization, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                              (255, 255, 255), 1)
                
                #Timer text
                cv2.putText(visualization, timer_text, (bar_x, bar_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
        #Return whether hand raising is confirmed (after the required duration)
        return self.confirmed_hand_raised
    
    def calculate_distance(self, depth_map, person_mask):
        """
        Calculate the distance to the person using the depth map
        
        Note: The iPhone depth data is already in meters
        """
        #Ensure person_mask has the same dimensions as depth_map
        if person_mask.shape != depth_map.shape:
            person_mask = cv2.resize(person_mask, (depth_map.shape[1], depth_map.shape[0]), 
                                     interpolation=cv2.INTER_NEAREST)
        
        #Extract depth values for the person
        person_depth = depth_map[person_mask > 0]
        
        #Filter out invalid depth values (too close or too far)
        valid_depths = person_depth[(person_depth > 0.1) & (person_depth < 5.0)]  # 0.1m to 5m range
        
        if len(valid_depths) == 0:
            return None
        
        #Calculate the 25th percentile depth to get closer distance (person's front)
        distance = np.percentile(valid_depths, 25)
        
        return distance  #Already in meters
    
    def segmentation(self, rgb_image):
        """
        Create a mask for the person using MediaPipe's Selfie Segmentation
        """
        #Process the RGB image with MediaPipe Selfie Segmentation
        results = self.selfie_segmentation.process(rgb_image)
        
        #Get the segmentation mask
        segmentation_mask = results.segmentation_mask
        
        if segmentation_mask is None:
            #Return an empty mask if segmentation failed
            return np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        
        #Convert the floating point mask to a binary mask
        binary_mask = (segmentation_mask > 0.1).astype(np.uint8) * 255
        
        #Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
        
        return binary_mask
    
    def prepare_confidence_visualization(self, confidence_map):
        """
        Prepare confidence map for visualization
        """
        if confidence_map is None or confidence_map.shape[0] == 0 or confidence_map.shape[1] == 0:
            return None
            
        #Scale confidence to 0-255
        conf_viz = confidence_map.copy() * 255
        conf_viz = conf_viz.astype(np.uint8)
        
        #Apply colormap for better visualisation
        conf_viz = cv2.applyColorMap(conf_viz, cv2.COLORMAP_HOT)
        
        label_text = "Depth Confidence Map"
        cv2.putText(conf_viz, label_text, (10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return conf_viz

    def start_processing_stream(self):
        while True:
            self.event.wait()  #Wait for new frame to arrive

            try:
                #Copy the newly arrived RGBD frame
                depth = self.session.get_depth_frame()
                rgb = self.session.get_rgb_frame()
                confidence = self.session.get_confidence_frame()
                
                #Print frame dimensions occasionally for debugging
                if self.debug_mode and np.random.random() < 0.01:  # 1% chance to print
                    print(f"RGB shape: {rgb.shape}, Depth shape: {depth.shape}")
                    if confidence is not None and confidence.shape[0] > 0:
                        print(f"Confidence shape: {confidence.shape}, range: {confidence.min():.2f}-{confidence.max():.2f}")
                    if depth is not None and depth.size > 0:
                        print(f"Depth range: {depth.min():.2f}m to {depth.max():.2f}m, mean: {depth.mean():.2f}m")

                #Handle TrueDepth camera mirroring
                if self.session.get_device_type() == self.DEVICE_TYPE__TRUEDEPTH:
                    depth = cv2.flip(depth, 1)
                    rgb = cv2.flip(rgb, 1)
                    if confidence is not None and confidence.shape[0] > 0:
                        confidence = cv2.flip(confidence, 1)
                
                #STEP 1: Rotate all images 90 degrees BEFORE any processing
                rgb_rotated = self.rotate_image(rgb)
                depth_rotated = self.rotate_image(depth)
                
                if confidence is not None and confidence.shape[0] > 0 and confidence.shape[1] > 0:
                    confidence_rotated = self.rotate_image(confidence)
                else:
                    confidence_rotated = None
                
                #Get original dimensions for reference
                orig_height, orig_width = rgb_rotated.shape[:2]
                
                #STEP 2: Process the ROTATED image with MediaPipe
                rgb_rotated_rgb = rgb_rotated  # MediaPipe expects RGB
                results = self.pose.process(rgb_rotated_rgb)
                
                #STEP 3: Create person mask using segmentation
                person_mask = self.segmentation(rgb_rotated_rgb)
                
                #Ensure person_mask is the same size as the original image
                if person_mask.shape[:2] != (orig_height, orig_width):
                    person_mask = cv2.resize(person_mask, (orig_width, orig_height), interpolation=cv2.INTER_NEAREST)
                
                #STEP 4: Convert to BGR for visualization
                rgb_rotated_bgr = cv2.cvtColor(rgb_rotated, cv2.COLOR_RGB2BGR)
                
                #Create visualisation image
                visualization = rgb_rotated_bgr.copy()
                
                #STEP 5: Draw pose landmarks
                if results.pose_landmarks:
                    self.mp_drawing.draw_landmarks(
                        visualization,
                        results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS)
                
                #STEP 6: Detect if hand is raised with time-based confirmation
                hand_raised = self.detect_hand_raised(results.pose_landmarks, visualization)
                
                #STEP 7: Calculate distance if person detected
                distance_m = None
                if np.sum(person_mask) > 0:  # If there is a person in the mask
                    distance_m = self.calculate_distance(depth_rotated, person_mask)
                
                #STEP 8: Prepare status text
                if results.pose_landmarks:
                    current_raised = (results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST].y < 
                                    (results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y + 
                                     results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y) / 2 - 0.05) or \
                                   (results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_WRIST].y < 
                                    (results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER].y + 
                                     results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER].y) / 2 - 0.05)
                                     
                    if hand_raised:
                        if distance_m is not None:
                            status_text = f"Hand Raised for 5s! Distance: {distance_m:.2f} m"
                            text_color = (0, 255, 0)  # Green
                        else:
                            status_text = "Hand Raised for 5s! Distance: unknown"
                            text_color = (0, 255, 255)  # Yellow
                    elif current_raised and self.hand_raised_start_time is not None:
                        # Hand is raised but not for enough time
                        elapsed = time.time() - self.hand_raised_start_time
                        if distance_m is not None:
                            status_text = f"Raising hand... {elapsed:.1f}s. Distance: {distance_m:.2f} m"
                            text_color = (0, 255, 255)  # Yellow
                        else:
                            status_text = f"Raising hand... {elapsed:.1f}s. Distance: unknown"
                            text_color = (0, 255, 255)  # Yellow
                    else:
                        if distance_m is not None:
                            status_text = f"No Hand Raised. Distance: {distance_m:.2f} m"
                            text_color = (0, 0, 255)  # Red
                        else:
                            status_text = "Person detected, distance unknown"
                            text_color = (0, 0, 255)  # Red
                else:
                    status_text = "No Person Detected"
                    text_color = (0, 0, 255)  # Red
                
                #STEP 9: Add text and information to visualization
                cv2.putText(visualization, status_text, (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2)
                
                instr_text = "Raise either hand above chest level for 5 seconds to trigger detection"
                cv2.putText(visualization, instr_text, (20, visualization.shape[0] - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                #Draw distance bar if available
                if distance_m is not None:
                    bar_length = min(int(distance_m * 100), 200)  # Scale for visualization
                    cv2.rectangle(visualization, (20, 60), (20 + bar_length, 80), (0, 255, 255), -1)
                    cv2.rectangle(visualization, (20, 60), (220, 80), (255, 255, 255), 2)
                    
                    #distance text
                    cv2.putText(visualization, f"{distance_m:.2f}m", (20, 100), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                #STEP 10: Display the result
                cv2.imshow('Robot Waiter Vision', visualization)
                
                #Show depth visualization
                if depth_rotated is not None:
                    #Normalize and colorize depth map
                    depth_viz = depth_rotated.copy()
                    depth_viz = np.clip(depth_viz, 0, 5.0)  # Clip to 0-5m range
                    depth_viz = (depth_viz / 5.0 * 255).astype(np.uint8)
                    depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
                    
                    #Show depth map
                    cv2.imshow('Depth', depth_viz)
                    
                    #Always show confidence map if requested
                    if self.show_confidence and confidence_rotated is not None:
                        try:
                            conf_viz = self.prepare_confidence_visualization(confidence_rotated)
                            if conf_viz is not None:
                                cv2.imshow('Confidence', conf_viz)
                        except Exception as e:
                            print(f"Error displaying confidence map: {e}")
                    
                    #Show person mask in debug mode
                    if self.debug_mode:
                        try:
                            #Create a colorised person mask for better visualisation
                            person_mask_viz = cv2.cvtColor(person_mask, cv2.COLOR_GRAY2BGR)
                            cv2.putText(person_mask_viz, "Segmentation Mask", (10, 20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                            cv2.imshow('Person Mask', person_mask_viz)
                            
                            #Show depth for person only
                            if np.sum(person_mask) > 0:
                                #Ensure person_mask is the right shape for bitwise operations
                                if person_mask.shape[:2] != depth_viz.shape[:2]:
                                    resized_mask = cv2.resize(person_mask, (depth_viz.shape[1], depth_viz.shape[0]), 
                                                            interpolation=cv2.INTER_NEAREST)
                                else:
                                    resized_mask = person_mask
                                    
                                #Create a 3-channel mask
                                person_mask_3ch = cv2.merge([resized_mask, resized_mask, resized_mask])
                                person_mask_3ch = person_mask_3ch.astype(np.uint8)
                                
                                #Apply the mask
                                masked_depth = cv2.bitwise_and(depth_viz, person_mask_3ch)
                                cv2.imshow('Person Depth', masked_depth)
                        except Exception as e:
                            print(f"Error displaying debug visualizations: {e}")
                
            except Exception as e:
                print(f"Error processing frame: {e}")
            
            #Handle keyboard input
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to exit
                break
            elif key == ord('d'):  # 'd' key to toggle debug mode
                self.debug_mode = not self.debug_mode
                print(f"Debug mode {'ON' if self.debug_mode else 'OFF'}")
            elif key == ord('c'):  # 'c' key to toggle confidence display
                self.show_confidence = not self.show_confidence
                print(f"Confidence display {'ON' if self.show_confidence else 'OFF'}")
                if not self.show_confidence:
                    cv2.destroyWindow('Confidence')
            elif key == ord('t'):  # 't' key to adjust required time
                # Toggle between 1, 3, 5, and 10 seconds
                times = [1.0, 3.0, 5.0, 10.0]
                current_index = times.index(self.required_raise_duration) if self.required_raise_duration in times else 2
                next_index = (current_index + 1) % len(times)
                self.required_raise_duration = times[next_index]
                print(f"Required hand raise time: {self.required_raise_duration} seconds")
            
            self.event.clear()
    
    def __del__(self):
        #Release resources
        self.pose.close()
        self.selfie_segmentation.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    app = RobotWaiterVision()
    app.connect_to_device(dev_idx=0)
    app.start_processing_stream()
