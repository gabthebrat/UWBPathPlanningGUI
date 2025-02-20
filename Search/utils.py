import time, json, logging, cv2, os, math
import numpy as np
from shared_utils.shared_utils import normalize_angle, params, draw_pose_axes
from typing import List, Dict, Literal
from shared_utils.dronecontroller2 import DroneController

def validate_waypoints(json_filename):
    with open(json_filename, 'r') as f:
        data = json.load(f)
    
    valid = True
    for i, wp in enumerate(data['wp']): # wp is the datapoint for each waypoint (its coordinate, and distance/angle to next waypoint)
        # Check distance
        if wp['dist_cm'] < 20 and not i+1 == len(data['wp']):
            print(i, len(data['wp']))
            print(f"[WARNING] Waypoint {i+1} distance ({wp['dist_cm']}cm) is below minimum 20cm")
            valid = False
        if wp['dist_cm'] > 500:
            print(f"[INFO] Waypoint {i+1} distance ({wp['dist_cm']}cm) will be split into multiple commands")
        
        # Check angle
        if abs(wp['angle_deg']) > 360:
            print(f"[WARNING] Waypoint {i+1} angle ({wp['angle_deg']}°) exceeds 360 degrees")
            valid = False
    
        # Check for dist_cm = 0 condition (only valid if it is last waypoint; invalid otherwise.)
        if wp['dist_cm'] == 0 and i+1 == len(data['wp']):
            print(f"[INFO] Final Waypoint {i+1} distance is zero to indicate end of flight routine.")
        elif not wp['dist_cm'] == 0 and i+1 == len(data['wp']):
            print(f"[INFO] Final Waypoint {i+1} distance is NOT zero to indicate end of flight routine.") # TBC 7 Jan extra?
        elif wp['dist_cm'] == 0:
            print(f"[WARNING] Waypoint {i+1} distance is zero but is not the final waypoint. Invalid flight path.") # TBC 7 Jan extra?
            valid = False

    return valid

def scan_for_marker_bothsides(controller: DroneController, yaw_rate: int = 35, scan_deg: int = 90):
    """
    Scans for markers by rotating the drone both left and right.
    Stops and exits once a marker is detected.
    """
    scan_for_marker_oneside(controller, "left", yaw_rate, scan_deg)
    scan_for_marker_oneside(controller, "right", yaw_rate, scan_deg)
    logging.info("Finished scanning left and right, no markers detected.")

def scan_for_marker_oneside(controller: DroneController, direction: Literal["left", "right"], yaw_rate: int = 35, scan_deg: int = 90):
    """
    Rotates the drone in one direction (left or right) and scans for markers.
    If a marker is detected, stops the rotation and enters a separate function for detection and landing.
    """
    # Adjust yaw_rate for left or right direction
    yaw_rate = -abs(yaw_rate) if direction == "left" else abs(yaw_rate)
    
    # Store the initial heading at the start of the rotation
    scan_deg = min(scan_deg, 135)  # Ensures scan_deg doesn't exceed 135
    start_heading = controller.drone.get_yaw()
    abs_rotation = 0
    
    # Rotate and scan for markers
    while abs_rotation < scan_deg:
        detect_and_land(controller)     # IMPT: if detected, it will stay inside this function and land, never leaving!!
        controller.drone.send_rc_control(0, 0, 0, yaw_rate)
        time.sleep(0.2)
        current_heading = controller.drone.get_yaw()
        total_rotation = normalize_angle(current_heading - start_heading)
        abs_rotation = abs(total_rotation)
        logging.debug(f"Current heading: {current_heading} Rotation so far: {total_rotation}")
    
    # Stop the rotation
    controller.drone.send_rc_control(0, 0, 0, 0)
    current_heading = controller.drone.get_yaw()
    actual_rotation_diff = normalize_angle(current_heading - start_heading)
    logging.debug(f"Final heading after rotating: {current_heading}. Total rotation: {actual_rotation_diff}")

    # Rotate back to original heading
    if direction == "left":
        controller.drone.rotate_clockwise(abs_rotation)
    else:
        controller.drone.rotate_counter_clockwise(abs_rotation)
    
    current_heading = controller.drone.get_yaw()
    logging.info(f"Finished scanning {direction}, no markers detected. Start heading: {start_heading}. Reverting to heading: {current_heading}.")
    time.sleep(1)
        
def detect_and_land(controller:DroneController):

    """
    Internal; called by scanning function.

    TBC 18 Feb - Should find a way to stay inside this loop forever, once the drone detected its first obstacle. Maybe with a RC yaw if it loses sight of the target.
    """
    marker_latch = False    # once marker_found, latch and don't exit the while loop immediately, even if it loses detection.
    approach_complete = False
    centering_complete = False
    centering_threshold = 30    # in px

    while not approach_complete:
        # Check for markers
        frame = controller.get_current_frame()
        display_frame = frame.copy()
        marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame, display_frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
        if marker_found:  
            # Draw marker detection and pose information on the ONE detected valid marker
            spin_count = 0      
            display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
            logging.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
            logging.debug(f"Marker detected: {marker_id}. Available: {controller.marker_client.is_marker_available(marker_id)}. Currently locked on: {controller.markernum_lockedon}")
            marker_latch = True

            # PART 1: DETECTION AND LOCK-ON LOGIC
                # Difficult logic, discussed between Yaqub and Gab 11 Feb
                # Also works without a server, since .is_marker_available will return True
                # Can be tested remotely; Has worked well caa 14 Feb
            if controller.marker_client.is_marker_available(marker_id) or marker_id == controller.markernum_lockedon:

                if controller.markernum_lockedon is None or marker_id == controller.markernum_lockedon: # first time detecting an available marker, or subsequent time detecting a marker locked on by it (but shown as no longer available)
                    controller.markernum_lockedon = marker_id
                    controller.marker_client.send_update(marker_id=marker_id, detected=True, status_message=f"Locked on {controller.markernum_lockedon}") 
                    logging.info(f"Locked onto {controller.markernum_lockedon}! Switching to approach sequence...")
                    controller.drone.send_rc_control(0, 0, 0, 0)
                    # time.sleep(1) # for stabilisation? but also slows down video feed
                    goto_approach_sequence = True
                    cv2.putText(display_frame, f"LOCKED ON: {controller.markernum_lockedon}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                elif centering_complete and marker_id != controller.markernum_lockedon:   
                    # centering complete, but lost detection and detected another valid and available marker
                    # Do NOT switch lock-on anymore! Go back to the start of the code, and hopefully restore lock-on.
                    logging.debug(f"Lost locked-on marker during centering. Finding marker again.")
                    centering_complete = False  # 13 Feb TBC Gab
                    continue
                
                else:   # centering incomplete, but lost detection halfway and detected something else. Switch lock-on.
                    logging.debug(f"Switching lock-on from {controller.markernum_lockedon} to {marker_id}...")
                    controller.marker_client.send_update(marker_id=controller.markernum_lockedon, detected=False, status_message=f"Switch lock-on from {controller.markernum_lockedon} to {marker_id}") 
                    controller.markernum_lockedon = marker_id   # locking onto the new one
                    controller.marker_client.send_update(marker_id=controller.markernum_lockedon, detected=True)
                    goto_approach_sequence = False

            else: # marker detected is NOT available
                logging.info(f"Valid marker {marker_id} is NOT available (and not already locked-on previously).")
                goto_approach_sequence = False
            
            # PART 2: APPROACH SEQUENCE LOGIC
                # Still work in progress caa 20 Feb
                # Can only be tested properly IRL
            if goto_approach_sequence is True and not params.NO_FLY:                  
                logging.info(f"Still locked on and centering/approaching marker {controller.markernum_lockedon}...")
                
                # Center on the marker
                frame_center = frame.shape[1] / 2
                marker_center = np.mean(corners[0], axis=0)
                x_error = marker_center[0] - frame_center
                
                if not centering_complete:
                    if abs(x_error) > centering_threshold:
                        # Calculate yaw speed based on error
                        yaw_speed = int(np.clip(x_error / 5, -25, 25))      # TODO: Needs fine-tuning
                        controller.drone.send_rc_control(0, 0, 0, 0)
                        time.sleep(0.5)
                        controller.drone.send_rc_control(0, 0, 0, yaw_speed)
                        logging.info(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                    else:
                        logging.info(f"Marker centered! Error = {x_error:.1f}. Starting approach...")
                        controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                        time.sleep(1)  # Stabilize
                        centering_complete = True
                
                elif not approach_complete:
                    current_distance_3D = controller.get_distance()
                    if current_distance_3D is None:
                        logging.info("Lost marker during approach...")  # should not reach here! caa 13 Feb
                        time.sleep(0.1)
                        continue

                    current_height = controller.drone.get_distance_tof() - params.EXTRA_HEIGHT
                    current_distance_2D = np.sqrt(current_distance_3D**2 - current_height**2)

                    logging.info(f"3D distance to marker: {current_distance_3D:.1f}cm | Drone height: {current_height:.1f}cm | 2D distance to marker: {current_distance_2D:.1f}cm")
                    controller.marker_client.send_update(status_message=f"Centered! Approaching marker {controller.markernum_lockedon}")
                    cv2.putText(display_frame, "Centering Complete. Approaching...", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 4)

                    if current_distance_2D >= 500:
                        step_dist:int = 150
                        logging.info(f"{current_distance_2D:.2f}cm distance too large. Stepping forward {step_dist}cm to approach marker...")
                        # controller.drone.send_rc_control(0, 0, 0, 0)
                        controller.drone.move_forward(step_dist)
                        time.sleep(0.5)  # Wait for movement to complete
                        centering_complete = False
                        continue        # re-enter the loop; need to re-detect marker and re-measure distance. 

                    elif current_distance_2D > 0 and current_distance_2D < 500:
                        logging.info(f"Final Approach: Moving forward {int(current_distance_2D)}cm to marker.")
                        controller.drone.move_forward(int(current_distance_2D))
                        logging.info("Approach complete!")
                        controller.drone.send_rc_control(0, 0, 0, 0)
                        approach_complete = True
                        controller.marker_client.send_update(marker_id=marker_id, landed=True, status_message=f"Landed on {marker_id}")
                        time.sleep(1)
                        break

            elif goto_approach_sequence is True and params.NO_FLY == True: # if simulating
                logging.info("Valid marker found! Not centering and approaching, since not flying. Program continues.")

            else:   #goto_approach_sequence is False
                logging.info("Marker found, but not centering and approaching yet.")

            controller.set_display_frame(display_frame)

        elif marker_latch and spin_count < 20:  # no marker found now, but was once detected!
            logging.info("Marker previously found, but lost. Staying put and spinning.")    # TODO TBC should return to main search path
            controller.marker_client.send_update(marker_id=controller.markernum_lockedon, detected=False, status_message=f"Spinning; Finding lost marker {controller.markernum_lockedon}")
            controller.set_display_frame(display_frame)    
            controller.drone.send_rc_control(0,0,0,5)
            spin_count += 1
            continue
        
        else:   # no marker found even after spinning 20 times
            # TBC 20 Feb - how to proceed with search path after this? 
            logging.info("No marker found. Lock-on reset. Proceed with search.")
            controller.markernum_lockedon = None    # reset to None in case it got lost (probably extra but redundancy is good)
            # controller.marker_client.send_update(status_message=f"No marker found. Lock on reset. Proceed with search")
            controller.set_display_frame(display_frame)
            break
    
    if approach_complete:
        logging.info("Landing on victim. (see utils: detect_and_land)") 
        controller.drone.end()