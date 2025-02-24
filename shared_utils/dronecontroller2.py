import torch
import cv2
import time
from cv2 import aruco
import numpy as np
import math
import threading
from threading import Lock, Event
import logging  # in decreasing log level: debug > info > warning > error > critical
from typing import List, Dict


from .customtello import CustomTello, MockTello
from markerserver.swarmserverclient import MarkerClient
from .shared_utils import *

"""
17 Feb LATEST Testing - V2 for Simultaneous takeoff and integrating ALL controllers
"""

class DroneController:
    """
    No takeoff / flying / landing commands takes place here. (caa 17 Feb)
    """
    def __init__(self, network_config, drone_id, laptop_only = False, load_midas = True, imshow = True):
        # Initialize Tello
        logging.debug(f"Laptop only (bool): {laptop_only}")
        self.drone = MockTello() if laptop_only else CustomTello(network_config)
        
        if load_midas:
            # Initialize MiDaS model
            self.model_type = "MiDaS_small"
            self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
            self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
            self.midas.to(self.device)
            self.midas.eval()
            
            # Load MiDaS transform
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform

        ## COMMENT OUT BELOW FOR TESTING

        self.drone.connect()
        logging.info(f"Start Battery Level: {self.drone.get_battery()}%")
        self.drone.streamon()

        if not laptop_only:
            # Set video stream properties to reduce latency
            self.drone.set_video_resolution(self.drone.RESOLUTION_480P)     # IMPT: Default 720P - need to use correct calibration params if set to 480P 
            self.drone.set_video_fps(self.drone.FPS_15)
            self.drone.set_video_bitrate(self.drone.BITRATE_3MBPS)
        
        # 19 FEB NEW Video Stream Properties
        self.frame_reader = self.drone.get_frame_read()
        self.current_frame = None
        self.display_frame = None
        self.frame_lock = Lock()
        self.stream_thread = None
        self.stop_event = Event()
        time.sleep(2)

        self.start_video_stream(imshow=imshow)

        ## COMMENT OUT ABOVE FOR TESTING

        # Color depth map
        self.depth_map_colors = {}
        
        # Controller state
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None        # 29 Jan Gab: This is the 3D distance - decently accurate
        self.distance_lock = Lock()
        self.is_running = True
        self.marker_detected = False
        self.markernum_lockedon:int = None
        self.is_centered = False
        self.movement_completed = False
        self.valid_ids = set(range(1, 9))   # set because doesn't change
        self.invalid_ids = []               # detected/landed by other drones, updated by MarkerClient
        self.danger_ids = set(range(9, 15)) # set because doesn't change
        self.exit_ids = set(range(50, 100)) # set because doesn't change; for drone to avoid exiting search area
        self.exit_detected = False
        self.exit_distance_3D = None
        self.marker_positions = {}
         
        # Navigation parameters
        self.drone_id = drone_id
        self.move_speed = 20
        self.yaw_speed = 50
        self.forward_tof_dist = None # to be updated by subthread
        self.forward_tof_lock = Lock()

        # 18 FEB NEW Searcher Navigation Parameters
        self.waypoints_executed:List[Dict] = []
        self.my_current_pos:tuple = (0,0)       # Dead reckoning
        self.my_current_orientation:float = 0   # Dead reckoning
        self.my_imu_orientation = 0             
        self.my_start_pos:tuple = (0,0)     # CAN BE A PARAM
        self.status:str = None              # should be a property??
        
        self.target_yaw = None      # TESTING END-JAN - for exit marker

        # Swarm Server/Client (17 Feb new)
        self.marker_client = MarkerClient(drone_id = drone_id)

    # 19 FEB NEW VIDEO HANDLING

    def start_video_stream(self, imshow:bool = True):
        """Start the video streaming in a separate thread"""
        self.stop_event.clear()
        self.stream_thread = threading.Thread(target=self._stream_video, args={imshow,})
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
    def stop_video_stream(self):
        """Stop the video streaming thread"""
        self.stop_event.set()
        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2)
            
    def get_current_frame(self):
        """Thread-safe method to get the latest frame. External method."""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
        
    def get_display_frame(self):
        """Thread-safe method to get the display frame. Internal method."""
        with self.frame_lock:
            return self.display_frame.copy() if self.display_frame is not None else None
        
    def set_display_frame(self, frame):
        """Thread-safe method to set the display frame. External method."""
        with self.frame_lock:
            self.display_frame = frame.copy() if frame is not None else None
            
    def _stream_video(self, imshow:bool = True):
        """Video streaming thread function"""
        while not self.stop_event.is_set():
            try:
                # Capture new frame
                frame = capture_frame(self.frame_reader)
                if frame is None:
                    continue
                    
                # Store frame thread-safely
                with self.frame_lock:
                    self.current_frame = frame.copy()
                
                # Display the frame
                display_frame = self.get_display_frame()
                if display_frame is not None and imshow:
                    cv2.imshow(f"Drone View {self.drone_id}", display_frame)

                # Handle keyboard input
                key = cv2.waitKey(1)
                if key != -1:
                    if key == ord('q'):
                        logging.info("Quit command received")
                        self.stop_event.set()
                        break
                    elif key == ord('l'):
                        logging.debug("DEBUG LOGPOINT")
                    # Handle any additional key commands
                    self.handle_key_command(key)

            except Exception as e:
                logging.error(f"Error in video stream: {e}")
                time.sleep(0.1)
                
        self._cleanup()
            
    def handle_key_command(self, key):
        """Handle keyboard commands - can be overridden by subclasses"""
        pass
        
    def _cleanup(self):
        """Clean up resources when video stream ends"""
        cv2.destroyAllWindows()
        logging.info("Destroying all windows.")
        for i in range(4):
            cv2.waitKey(1)
            
        # Shutdown drone
        self.drone.streamoff()
        self.drone.land()
        self.drone.end()
        
    def shutdown(self):
        """Properly shutdown the controller"""
        self.stop_video_stream()
        self._cleanup()

    ## END OF NEW VIDEO 
    
    def takeoff_simul(self, drones_list:list):
        """
        Waits for a takeoff signal before taking off.
        """
        self.marker_client.send_takeoff_request(drones_list)
        while not self.marker_client.takeoff_signal:
            time.sleep(0.1)  # Wait for takeoff command
        logging.info(f"Tello {self.drone_id} is taking off!")
        self.drone.takeoff()

    def generate_color_depth_map(self, frame):
        """Process frame through MiDaS to get depth map"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(frame_rgb).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX)
        return cv2.applyColorMap((depth_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
    def process_depth_color_map(self, depth_colormap):
        """
        :param: depth_colormap: the actual frame of the depth color map
        :return: None
        Split depth map into regions for navigation. Updates class attributes.
        """
        h, w = depth_colormap.shape[:2]
        left_region = depth_colormap[:, :w//3]
        center_region = depth_colormap[:, w//3:2*w//3]
        right_region = depth_colormap[:, 2*w//3:]
        
        # Draw region divisions on depth map
        cv2.line(depth_colormap, (w//3, 0), (w//3, h), (0, 255, 0), 2)
        cv2.line(depth_colormap, (2*w//3, 0), (2*w//3, h), (0, 255, 0), 2)
        
        # Analyze colors in regions
        red_left = np.sum((left_region[:, :, 2] > 150) & (left_region[:, :, 0] < 50))
        red_center = np.sum((center_region[:, :, 2] > 150) & (center_region[:, :, 0] < 50))
        red_right = np.sum((right_region[:, :, 2] > 150) & (right_region[:, :, 0] < 50))
        
        blue_left = np.sum((left_region[:, :, 0] > 150) & (left_region[:, :, 2] < 50))
        blue_center = np.sum((center_region[:, :, 0] > 150) & (center_region[:, :, 2] < 50))
        blue_right = np.sum((right_region[:, :, 0] > 150) & (right_region[:, :, 2] < 50))

        # Update class attribute
        self.depth_map_colors["red"] = {
            "left": red_left,
            "center": red_center,
            "right": red_right
        }

        self.depth_map_colors["blue"] = {
            "left": blue_left,
            "center": blue_center,
            "right": blue_right
        }

    def get_distance(self):
        with self.distance_lock:
            return self.distance
            
    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance

    def get_tof_distance(self):
        with self.forward_tof_lock:
            return self.forward_tof_dist
        
    def update_current_pos(self, rotation_deg:float=0, distance_cm:float=0):
        """
        TODO 19 FEB - doesnt work; can also take in an argument to set the actual current pos? inspo PPFLY2

        :args:
        rotation_deg: input given in as GUI json (i.e. +ve ccw) 

        (Dead reckoning) Update the drone's position and store it in the waypoints_executed list.
        Also stores the distance travelled / degrees rotated in the previous command.
        Assumes drone rotates and only travels forward in straight lines.
            Drone's rotation is +ve in the clockwise direction (i.e. turning right)
            GUI's rotation (and by extension, waypoints.json) is +ve in the anticlockwise direction (i.e. turning left)
        
        """
        rad = math.radians(rotation_deg)
        new_x = self.my_current_pos[0] + int(distance_cm * math.sin(rad))
        new_y = self.my_current_pos[1] + int(distance_cm * math.cos(rad))

        self.my_current_pos = (round(new_x,2), round(new_y,2))
        self.my_current_orientation -= rotation_deg
        self.my_imu_orientation = self.drone.get_yaw()

        if self.my_current_orientation > 180:
            self.my_current_orientation -= 360
        elif self.my_current_orientation < -180:
            self.my_current_orientation += 360

        self.waypoints_executed.append({"x_cm": self.my_current_pos[0], "y_cm": self.my_current_pos[1], 
                                        "orientation_deg": self.my_current_orientation, "imu_orientation_deg": self.my_imu_orientation,
                                        "distance_cm": distance_cm, "rotation_deg": rotation_deg})
        logging.info(f"Updated waypoints executed: {self.waypoints_executed}")

    def detect_markers(self, frame, display_frame, marker_size=14.0):
        """
        Detect ArUco markers and estimate pose.
        Returns values of the FIRST DETECTED VALID MARKER.

        Additionally, calculates the distance between the detected marker and a danger marker if present.

        :return:
            marker_detected (bool), marker_corners, marker_id (int), rotation_vec, translation_vec
        """
        global CAMERA_MATRIX, DIST_COEFF
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        # Reset class attributes, for re-detection
        self.target_yaw = None
        self.danger_marker_distance = None 
        self.exit_detected = False
        self.exit_distance_3D = None 

        self.invalid_ids = self.marker_client.get_invalid_markers(self.valid_ids)
        logging.debug(f"Invalid markers: {self.invalid_ids}")

        if ids is not None:
            detected_ids = ids.flatten()
            logging.debug(f"Detected IDs: {detected_ids}")

            # Check for exit markers - drone's subsequent behaviour is not coded here
            if set(detected_ids).intersection(self.exit_ids):
                self.exit_detected = True
                logging.info("Exit marker detected!")

            # Store first valid marker only
            valid_marker_info = {}   # only stores ONE valid marker info
            lockedon_marker_info = {} # either stores lockedon marker info or None
            danger_marker_info = {}  # Store all danger markers

            for i, marker_id in enumerate(detected_ids):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i].reshape(1, 4, 2), marker_size, CAMERA_MATRIX, DIST_COEFF
                )
                x, y, z = tvecs[0][0]
                euclidean_distance = np.sqrt(x*x + y*y + z*z)

                if marker_id in self.valid_ids and (marker_id not in self.invalid_ids or marker_id == self.markernum_lockedon):
                    if not valid_marker_info:  # Ensures only first valid marker is stored
                        valid_marker_info = {
                            "id": int(marker_id),  # Ensure ID is a Python int
                            "position": (x, y, z),
                            "distance": euclidean_distance,
                            "corners": corners[i],
                            "rvecs": rvecs[0],
                            "tvecs": tvecs[0]
                        }

                    if marker_id == self.markernum_lockedon:    # Stores marker info of previously locked on ID
                        lockedon_marker_info = {
                            "id": int(marker_id),  # Ensure ID is a Python int
                            "position": (x, y, z),
                            "distance": euclidean_distance,
                            "corners": corners[i],
                            "rvecs": rvecs[0],
                            "tvecs": tvecs[0]
                        }
                elif marker_id in self.danger_ids:  # Store danger marker details
                    danger_marker_info[marker_id] = {
                        "position": (x, y, z),
                        "distance": euclidean_distance,
                        "corners": corners[i],
                        "rvecs": rvecs[0],
                        "tvecs": tvecs[0]
                    }
                # If it's an exit marker, compute its distance and yaw
                elif marker_id in self.exit_ids:
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    self.exit_distance_3D = euclidean_distance
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvecs[0])
                    # Calculate yaw (rotation around Z-axis)
                    yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw in radians
                    self.target_yaw = np.degrees(yaw)
                    logging.debug(f"Exit marker yaw: {self.target_yaw:.2f}°")

            if lockedon_marker_info:    # ie. if lockedon_marker is still detected, even if its not the first detection, discard the valid_marker in favour of locked_on marker
                valid_marker_info = lockedon_marker_info

            # Compute distance to the nearest danger marker (if any)
            shortest_danger_distance = float("inf")  
            shortest_id = None
            closest_danger_marker = None  

            if valid_marker_info:
                for danger_id, danger_data in danger_marker_info.items():
                    dx = valid_marker_info["position"][0] - danger_data["position"][0]
                    dy = valid_marker_info["position"][1] - danger_data["position"][1]
                    dz = valid_marker_info["position"][2] - danger_data["position"][2]
                    computed_distance = np.sqrt(dx*dx + dy*dy + dz*dz)

                    if computed_distance < shortest_danger_distance:
                        shortest_danger_distance = computed_distance
                        shortest_id = danger_id
                        closest_danger_marker = danger_data  

                if shortest_id is not None:
                    self.danger_marker_distance = shortest_danger_distance
                    logging.info(f"Danger marker {shortest_id} detected. Distance: {shortest_danger_distance:.2f}m")

                    # Compute marker center
                    marker_center = tuple(map(int, np.mean(closest_danger_marker["corners"][0], axis=0)))

                    # Draw marker annotations
                    cv2.circle(display_frame, marker_center, 10, (0, 0, 255), -1)  # Red dot for danger marker

                    cv2.polylines(display_frame, 
                                [closest_danger_marker["corners"].astype(np.int32)], 
                                True, (0, 0, 255), 2)  # Red bounding box

                    cv2.putText(display_frame, 
                                f"ID: {shortest_id} Dist to {valid_marker_info['id']}: {shortest_danger_distance:.2f}m", 
                                (marker_center[0], marker_center[1] - 20),  
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    # Draw coordinate axes for the danger marker
                    cv2.drawFrameAxes(display_frame, CAMERA_MATRIX, DIST_COEFF, 
                                    closest_danger_marker["rvecs"], 
                                    closest_danger_marker["tvecs"], 
                                    10)

            # Return the first valid marker detected
            if valid_marker_info:
                self.set_distance(valid_marker_info["distance"])
                return True, valid_marker_info["corners"], valid_marker_info["id"], valid_marker_info["rvecs"], valid_marker_info["tvecs"]

        self.set_distance(None)
        return False, None, None, None, None