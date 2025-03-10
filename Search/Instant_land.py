import threading
import sys
from djitellopy import Tello
import cv2
import time
import keyboard
import cv2.aruco as aruco
import numpy as np
import json
import math
np.set_printoptions(legacy='1.25')

class CustomTello(Tello):
    def __init__(self, 
                 host='192.168.0.108',  # First drone's IP
                 control_port=9008,      # Keep standard Tello port
                 state_port=8008,        
                 video_port=11108):       # Unique video port
        
        # Store custom configuration
        self.TELLO_IP = host
        self.CONTROL_UDP_PORT = control_port
        self.STATE_UDP_PORT = state_port
        self.VS_UDP_PORT = video_port
        
        Tello.STATE_UDP_PORT = state_port
        Tello.CONTROL_UDP_PORT = control_port
        
        # Call parent's init with our custom host
        super().__init__(host)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = video_port

id = 0
dis = [1 for i in range(50)]
ang = [1 for i in range(50)]
pos = [0 for i in range(2)] # Position of drone [x, y]
marker_located = [0 for i in range(50)] # 0 for not located, 1 for located
marker_located[0] = 1
marker_list = []
status = ""
course = 0

LAND_ID = 0 # set to 0 for no land
FLYING_STATE = False
waypoints = [] # to store executed waypoints and drone's current position

# Load in the calibration data
calib_data_path = "../calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 6  # centimeters (measure your printed marker size)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()
stream_ready = threading.Event()

###########################################################################################################

def scan_for_marker(drone):
    
    global ang, heading, dis, marker_IDs, status, course

    #course = heading

    if course + 90 < 180:
        status = f"Scanning right... (case 1, current heading = {course})"
        while heading < 60 + course:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
                    #marker_located[id] = 1
            drone.send_rc_control(0, 0, 0, 50)
        drone.send_rc_control(0, 0, 0, 0)
    else:
        status = f"Scanning right... (case 2, current heading = {course})"
        while heading < course - 300 or (heading > course - 150 and heading < course + 60):
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
                    #marker_located[id] = 1
            drone.send_rc_control(0, 0, 0, 50)
        drone.send_rc_control(0, 0, 0, 0)
    
    #time.sleep(0.5)

    if course - 90 > -180 and course + 90 < 180:
        status = f"Scanning left... (case 3, current heading = {course})"
        while heading > -60 + course:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, -50)
        drone.send_rc_control(0, 0, 0, 0)
    elif course + 90 >= 180:
        status = f"Scanning left... (case 4, current heading = {course})"
        while heading > -60 + course  or heading < course - 200:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, -50)
        drone.send_rc_control(0, 0, 0, 0)
    else:
        status = f"Scanning left... (case 5, current heading = {course})"
        while heading > course + 300 or heading < course + 150:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, -50)
        drone.send_rc_control(0, 0, 0, 0)

    status = "Left side scan complete. Reorienting to face forward"
    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)
    
    time.sleep(1)

    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)
    
    status = ""

def scan_for_marker_left(drone):
    
    global ang, heading, dis, marker_IDs, status, course

    #course = heading

    if course - 90 > -180:
        status = f"Scanning left... (case 3, current heading = {course})"
        while heading > -60 + course:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, -50)
        drone.send_rc_control(0, 0, 0, 0)
    else:
        status = f"Scanning left... (case 4, current heading = {course})"
        while heading > course + 300 or heading < course + 150:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, -50)
        drone.send_rc_control(0, 0, 0, 0)

    status = "Left side scan complete. Reorienting to face forward"
    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)
    status = ""

    time.sleep(1)

    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)

    status = ""

def scan_for_marker_right(drone):
    
    global ang, heading, dis, marker_IDs, status, course

    #course = heading

    if course + 90 < 180:
        status = f"Scanning right... (case 1, current heading = {course})"
        while heading < 60 + course:
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, 50)
        drone.send_rc_control(0, 0, 0, 0)
    else:
        status = f"Scanning right... (case 2, current heading = {course})"
        while heading < course - 300 or (heading > course - 150 and heading < course + 60):
            for id in marker_list:
                if marker_located[id] == 0:
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
            drone.send_rc_control(0, 0, 0, 50)
        drone.send_rc_control(0, 0, 0, 0)

    status = "Right side scan complete. Reorienting to face forward..."
    
    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)

    time.sleep(1)

    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)
    
    status = ""

def locate_marker(drone,id): # Aligns with marker and measures its position
    
    global ang, heading, dis, status, marker_located

    dis[id] = 0
    time.sleep(0.5)
    if dis[id] == 0: # In the case where there are 3 markers in sight but last marker becomes out of sight when measuring second last
        drone.rotate_clockwise(int(ang[id]*2.5))
    ang[id] = 0 # Reset angle for marker #id
    time.sleep(0.5)
    cycle = 0
    status = f"Aligning with marker {id}"
    while abs(ang[id]) >1.5: # Aligning with marker #id
        if cycle <2 :
            drone.rotate_clockwise(int(ang[id]*1.2))
        else:
            drone.rotate_clockwise(int(ang[id]*1.5))
        cycle += 1
        ang[id] = 0 # Reset angle for each cycle
        time.sleep(0.5)
        if cycle == 5:
            break
    dis[id] = 0
    print(f"Calculating Marker {id}'s position...")
    status = f"Calculating Marker {id}'s position..."
    time.sleep(0.5)
    if dis[id] == 0:
        print(f"Marker {id} was lost")
        status = f"Marker {id} was lost"
        marker_located[id] = 0
        marker_list.pop(marker_list.index(id))
    else:
        drone.move_forward(int(dis[id]))
        drone.land()

def ascend(drone,altitude):
    
    global height, status

    if abs(height-altitude)>10:
        drone.go_xyz_speed(0,0,int(altitude-height),100)
        print(f"Ascending to height {altitude}")
        status = f"Ascending to height {altitude}"
    else:
        print(f"Already at {height}")
        status = f"Already at {height}"


def stream_video(drone):
    
    global heading, pos, ang, height, marker_IDs, marker_list, status, dis, id, sys, course

    while True:
        ret = True
        frame1 = drone.get_frame_read().frame
        frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
        height = drone.get_height()
        battery = drone.get_battery()
        heading = drone.get_yaw()
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        
        cv2.putText(frame,f"Height: {height} cm | Heading: {heading} degrees | Course: {course} degrees | Battery: {battery} %",(10,20),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
        cv2.putText(frame,f"Position: X: {round(pos[0], 2)} Y: {round(pos[1], 2)} | Status: {status}",(10,40),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            #cv2.putText(frame,f"Number of markers detected: {marker_IDs.size} ",(10,40),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                if ids[0] not in marker_list and ids[0] != 0 :
                    marker_list.append(ids[0])
                    print(f"New marker {ids[0]} is detected")
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )

                scaling_factor = 2.15
                #scaling_factor = 2.5
                actual_distance = np.sqrt((scaling_factor*distance)**2 - height**2)

                id = ids[0]
                if ids[0]>= 50:
                    print(f"id: {ids[0]} is out of list range")
                    continue
                #if dis[ids[0]] == 0:
                    #dis[ids[0]] = actual_distance
                dis[ids[0]] = actual_distance
                angle = 20/(30+(actual_distance-150)/5)*(round(tVec[i][0][0],1))

                ang[ids[0]] = angle
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv2.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(actual_distance, 2)} Ang: {round(angle,2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                
                # print(ids, "  ", corners)

        cv2.imshow("frame", frame)
    
        # Check if streaming readiness hasn't been signaled yet
        if not stream_ready.is_set():
            # Signal that video streaming is ready
            stream_ready.set()
            print("Event Signal Set: Stream is live.")

        if cv2.waitKey(1) & 0xFF == ord('z'):
            break

    #cap.release()
    cv2.destroyAllWindows()
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(1)
    drone.land()
    sys.exit()

###########################################################################################################

def validate_waypoints():
    with open('waypoint.json', 'r') as f:
        data = json.load(f)
    
    valid = True
    for i, wp in enumerate(data['wp']):
        # Check distance
        if wp['dist_cm'] < 20:
            print(f"Warning: Waypoint {i+1} distance ({wp['dist_cm']}cm) is below minimum 20cm")
            valid = False
        if wp['dist_cm'] > 500:
            print(f"Note: Waypoint {i+1} distance ({wp['dist_cm']}cm) will be split into multiple commands")
        
        # Check angle
        if abs(wp['angle_deg']) > 360:
            print(f"Warning: Waypoint {i+1} angle ({wp['angle_deg']}°) exceeds 360 degrees")
            valid = False
    
    return valid

def execute_waypoints(drone):
    global LAND_ID, FLYING_STATE, waypoints, status,  course
    
    try:

        # Initialize position and orientation
        abs_position = {"x": 0, "y": 0}  # in cm
        orientation = 180  # Starting heading in degrees (assuming 180 as the initial heading)
        
        # Read waypoints from file
        with open('waypoint.json', 'r') as f:
            data = json.load(f)
        
        # Execute each waypoint
        for wp in data['wp']:
            # Handle rotation
            status = "Orienting"
            if wp['angle_deg'] != 0:
                orientation += wp['angle_deg']
                orientation %= 360  # Keep orientation within 0 to 360 degrees
                if orientation > 180:
                    orientation -= 360  # Convert to -180 to 180 range
                
                course = int(-orientation)+180
                if course > 180:
                    course = -(course-180)
                
                if wp['angle_deg'] < 0:
                    drone.rotate_clockwise(int(abs(wp['angle_deg'])))
                else:
                    drone.rotate_counter_clockwise(int(abs(wp['angle_deg'])))
                time.sleep(2)  # Wait for rotation to complete
                
                # Update position and record data after rotation
                update_position(waypoints, abs_position, orientation)
                
                if wp['angle_deg'] < 0:
                    scan_for_marker_right(drone)
                else:
                    scan_for_marker_left(drone)
            
            # Handle forward movement in 50 cm increments
            distance = wp['dist_cm']
            status = "Proceeding forward"
            if distance > 150:
                while distance > 100:
                    drone.move_forward(100)
                    update_position(waypoints, abs_position, orientation, 50)

                    scan_for_marker(drone)
                    time.sleep(1)

                    distance -= 100
                    time.sleep(0.5)
                
            # Move remaining distance (if between 50 and 100 cm)
            if distance > 20:
                drone.move_forward(distance)
                update_position(waypoints, abs_position, orientation, distance)

                scan_for_marker(drone)
                time.sleep(1)
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        if drone.is_flying:
            print("Landing...")
            status = "Landing..."
            drone.land()
        print("Mission completed!")
        
        # Save waypoints to JSON file
        with open("waypoints_commanded.json", "w") as f:
            json.dump(waypoints, f, indent=4)

def update_position(waypoints, position, orientation, distance=0):
    global pos
    """Update the drone's position and store it in the waypoints list."""
    rad = math.radians(orientation)
    pos[0] = position["x"] + int(distance * math.cos(rad))
    pos[1] = position["y"] + int(distance * math.sin(rad))
    position["x"] += int(distance * math.cos(rad))
    position["y"] += int(distance * math.sin(rad))
    waypoints.append({"x": position["x"], "y": position["y"], "orientation": orientation, "distance": distance})

###########################################################################################################

def flight_routine(drone):

    global dis, ang, pos, xpos, ypos, height, status
    print("waiting for event to signal video stream readiness")
    stream_ready.wait()
    print("event signaled for video stream readiness")
    status = "Taking off..."
    
    drone.takeoff()
    ascend(drone,100)
    drone.rotate_counter_clockwise(heading)
    execute_waypoints(drone)

def main():
    # Initialize the drone, connect to it, and turn its video stream on.
    drone = CustomTello()
    drone.connect()
    drone.streamon()
    print("drone connected and stream on. Starting video stream thread.\n")

    # Create and start the streaming thread
    stream_thread = threading.Thread(target=stream_video, args=(drone,))
    stream_thread.daemon = True

    # Start the thread
    stream_thread.start()

    # Execute the flight routine
    flight_routine(drone)

    print("Flight routine ended.")

    # Reboot the drone at the end
    #drone.reboot()

'''
if __name__ == "__main__":
    # Run the main function if this script is executed
    main()

'''
if __name__ == "__main__":
    print("Validating waypoints...")
    if validate_waypoints():
        print("Validation passed. Starting execution...")
        #execute_waypoints()
        main()
    else:
        print("Validation failed. Please check warnings above.")
