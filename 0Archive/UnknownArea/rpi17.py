import torch
import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import threading
from threading import Lock
import time

# Define configuration constants
NETWORK_CONFIG = {
    'host': '192.168.0.117',
    'control_port': 9017,
    'state_port': 8017,
    'video_port': 11117
}

def get_calibration_parameters():
    camera_matrix = np.array([
        [921.170702, 0.000000, 459.904354],
        [0.000000, 919.018377, 351.238301],
        [0.000000, 0.000000, 1.000000]
    ])
    dist_coeffs = np.array([0.036099, -0.028374, -0.003189, -0.001275, 0.000000])
    return camera_matrix, dist_coeffs

class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT

class DroneController:
    def __init__(self, network_config):
        # Initialize Custom Tello
        self.drone = CustomTello(network_config)
        self.drone.connect()
        print(f"Battery Level: {self.drone.get_battery()}%")
        self.drone.streamon()
        
        # Load MiDaS model locally
        self.model_type = "MiDaS_small"
        model_path = "midas_models/MiDaS_small.pt"
        
        print("Loading MiDaS model locally...")
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type, map_location="cpu")
        self.midas.load_state_dict(torch.load(model_path, map_location="cpu"))
        
        # Set up device
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        
        # Load MiDaS transform
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform
        
        # Controller state
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None
        self.distance_lock = Lock()
        self.marker_x = None
        self.marker_x_lock = Lock()
        self.is_running = True
        self.marker_detected = False
        self.is_centered = False
        self.movement_completed = False
        self.valid_ids = set(range(1, 9))
        self.invalid_ids = set(range(11, 15))
        self.marker_positions = {}
        
        # Navigation parameters
        self.move_speed = 20
        self.yaw_speed = 50
        
    def get_ext_tof(self) -> int:
        """Get ToF sensor reading"""
        response = self.drone.send_read_command("EXT tof?")
        try:
            return int(response.split()[1])
        except ValueError:
            return 8888
            
    def process_depth_map(self, frame):
        """Process frame through MiDaS to get depth map"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(frame_rgb).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX)
        return cv2.applyColorMap((depth_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
    def get_distance(self):
        with self.distance_lock:
            return self.distance
            
    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance
            
    def get_marker_x(self):
        with self.marker_x_lock:
            return self.marker_x
            
    def set_marker_x(self, x):
        with self.marker_x_lock:
            self.marker_x = x

    def detect_markers(self, frame, marker_size=15.0):
        """Detect ArUco markers and estimate pose"""
        camera_matrix, dist_coeffs = get_calibration_parameters()
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id[0] in self.valid_ids:
                    # Get pose estimation
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i].reshape(1, 4, 2), marker_size, camera_matrix, dist_coeffs
                    )
                    # Calculate Euclidean distance
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    # Store marker position and distance
                    marker_center = np.mean(corners[i][0], axis=0)
                    self.set_marker_x(marker_center[0])
                    self.set_distance(euclidean_distance)
                    return True, corners[i], marker_id[0], rvecs[0], tvecs[0]
        self.set_distance(None)
        self.set_marker_x(None)
        return False, None, None, None, None

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    if ids is not None:
        camera_matrix, dist_coeffs = get_calibration_parameters()
        
        cv2.drawFrameAxes(
            frame, camera_matrix, dist_coeffs, rvecs, tvecs, 10
        )
        
        rot_matrix = cv2.Rodrigues(rvecs)[0]
        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
        x, y, z = tvecs[0]
        roll, pitch, yaw = euler_angles
        
        euclidean_distance = np.sqrt(x*x + y*y + z*z)
        
        position_text = f"Pos (cm): X:{x:.1f} Y:{y:.1f} Z:{z:.1f}"
        rotation_text = f"Rot (deg): R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
        distance_text = f"Distance: {euclidean_distance:.1f} cm"
        
        cv2.putText(frame, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, rotation_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, distance_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame

def navigation_thread(controller):
    """Main navigation thread combining depth mapping and marker detection"""
    print("Starting navigation with depth mapping...")
    
    # Create single window for combined view
    cv2.namedWindow("Drone Navigation", cv2.WINDOW_NORMAL)
    
    # Ensure frame reader is initialized
    frame_reader = controller.drone.get_frame_read()
    time.sleep(2)  # Give time for camera to initialize
    
    # Initial movement
    #print("Moving to initial altitude...")
    #controller.drone.move_up(20)
    #time.sleep(5) #should be 2 but changing to 5 for swarming
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    approach_start_time = None
    
    try:
        print("Starting depth mapping and marker detection...")
        while controller.is_running:
            # Get frame with retry mechanism
            retry_count = 0
            frame = None
            while frame is None and retry_count < 3:
                frame = frame_reader.frame
                if frame is None:
                    print("Frame capture failed, retrying...")
                    time.sleep(0.1)
                    retry_count += 1
            
            if frame is None:
                print("Failed to capture frame after retries")
                continue
                
            # Create a copy of frame for visualization
            display_frame = frame.copy()
            
            # Get depth map
            depth_colormap = controller.process_depth_map(frame)
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame)
            if marker_found:
                # Draw marker detection and pose information
                marker_center = np.mean(corners[0], axis=0)
                cv2.circle(display_frame, 
                          (int(marker_center[0]), int(marker_center[1])), 
                          10, (0, 255, 0), -1)
                cv2.polylines(display_frame, 
                            [corners[0].astype(np.int32)], 
                            True, (0, 255, 0), 2)
                cv2.putText(display_frame, 
                          f"ID: {marker_id}", 
                          (int(marker_center[0]), int(marker_center[1] - 20)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw pose estimation
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                
                print(f"Valid marker {marker_id} detected! Switching to approach sequence...")
                
                # Center on the marker
                frame_center = frame.shape[1] / 2
                x_error = marker_center[0] - frame_center
                
                # Centering threshold
                centering_threshold = 30
                
                if not centering_complete:
                    if abs(x_error) > centering_threshold:
                        # Calculate yaw speed based on error
                        yaw_speed = int(np.clip(x_error / 10, -20, 20))
                        controller.drone.send_rc_control(0, 0, 0, yaw_speed)
                        print(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                    else:
                        print("Marker centered! Starting approach...")
                        controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                        time.sleep(1)  # Stabilize
                        centering_complete = True
                        approach_start_time = time.time()
                
                elif not approach_complete:
                    current_distance = controller.get_distance()
                    if current_distance is None:
                        print("Lost marker distance during approach...")
                        time.sleep(0.1)
                        continue
                        
                    print(f"Current distance to marker: {current_distance:.1f}cm")
                    
                    # Define safe approach distance (60cm from marker)
                    safe_distance = max(int(current_distance - 60), 0)  # Keep 60cm safety margin
                    
                    if safe_distance > 0:
                        print(f"Moving forward {safe_distance}cm to approach marker...")
                        controller.drone.send_rc_control(0, 0, 0, 0)  # Stop any existing movement
                        time.sleep(1)  # Stabilize
                        controller.drone.move_forward(safe_distance)  # Move exact distance
                        time.sleep(2)  # Wait for movement to complete
                        
                        # Verify new position
                        new_distance = controller.get_distance()
                        if new_distance is not None:
                            print(f"New distance to marker: {new_distance:.1f}cm")
                    
                    print("Approach complete!")
                    controller.drone.send_rc_control(0, 0, 0, 0)
                    approach_complete = True
                    time.sleep(1)
                    controller.drone.land()  # Final stabilization
                
                else:  # Both centering and approach are complete
                    print("Landing sequence initiated...")
                    controller.drone.land()
                    controller.marker_detected = True  # Now we can exit the loop
                    break
            
            # Split depth map into regions for navigation
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
            
            # Get ToF distance
            dist = controller.get_ext_tof()
            
            # Draw navigation info on display frame
            cv2.putText(display_frame, 
                      f"ToF: {dist}mm", 
                      (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      1, 
                      (0, 255, 0), 
                      2)
            
            # Navigation logic
            if not marker_found:
                if red_center > blue_center:
                    # Obstacle ahead - turn towards more open space
                    if blue_left > blue_right:
                        controller.drone.send_rc_control(0, 0, 0, -controller.yaw_speed)
                        time.sleep(2)  # Stabilize
                        cv2.putText(display_frame, "Turning Left", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        controller.drone.send_rc_control(0, 0, 0, controller.yaw_speed)
                        time.sleep(2)  # Stabilize
                        cv2.putText(display_frame, "Turning Right", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    if blue_center > red_center and dist <= 900: #dist 600
                        controller.drone.rotate_clockwise(135) #correct setting 135
                        time.sleep(2)  # Stabilize
                        cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
                        time.sleep(2)  # Stabilize
                        cv2.putText(display_frame, "Moving Forward", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Resize depth_colormap to match frame dimensions
            depth_colormap_resized = cv2.resize(depth_colormap, 
                                              (display_frame.shape[1]//2, display_frame.shape[0]))
            
            # Create combined view with original frame and depth map side by side
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels
            cv2.putText(combined_view, 
                      "Live Feed", 
                      (10, combined_view.shape[0] - 20), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      0.7, 
                      (255, 255, 255), 
                      2)
            cv2.putText(combined_view, 
                      "Depth Map", 
                      (display_frame.shape[1] + 10, combined_view.shape[0] - 20), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      0.7, 
                      (255, 255, 255), 
                      2)
            
            # Display combined view
            #cv2.imshow("Drone Navigation", combined_view)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"Navigation error: {e}")

    finally:
        print("Cleaning up navigation thread...")
        controller.drone.send_rc_control(0, 0, 0, 0)
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        time.sleep(0.5)       
        
def main():
    # Use the centralized network configuration
    controller = DroneController(NETWORK_CONFIG)
    
    try:
        print("Taking off...")
        controller.drone.takeoff()
        print("Moving to initial altitude...")
        controller.drone.move_up(20)
        time.sleep(5) #should be 2 but changing to 5 for swarming
        #time.sleep(2)
        
        # Start navigation thread
        nav_thread = threading.Thread(target=navigation_thread, args=(controller,))
        nav_thread.start()
        nav_thread.join()
        
    except Exception as e:
        print(f"Error in main: {e}")
        
    finally:
        controller.is_running = False
        controller.drone.land()
        controller.drone.streamoff()

if __name__ == "__main__":
    main()