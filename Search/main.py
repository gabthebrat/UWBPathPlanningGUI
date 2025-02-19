"""
python -m Search.main shared_params.params

Issues 19 Feb:
- Drone does not end when CV2 window closes
- Yet to test IRL
- update_current_pos does not work

"""

from shared_utils.dronecontroller2 import DroneController
from shared_utils.shared_utils import setup_logging, params
from .utils import *
import cv2
import threading
from threading import Lock, Event
import time
import logging
import sys

def execute_waypoints_scan_land(dronecontroller:DroneController, waypoint_json_w_ext):
    """
    Executes a given set of waypoints, scans as it is on the way, and lands 

    Scanning methodology:
    - Rotate first. If turned left, scan left (behind drone previously)
    - Next, move forward. Scans both sides after moving > 150cm

    """
    waypoints_success = True  # Flag to indicate successful execution of all waypoints
    
    try:       
        # Read waypoints from file
        with open(f'{waypoint_json_w_ext}', 'r') as f:
            data = json.load(f)
        
        # Process for each waypoint: Rotate first, then move forward in a straight line. 
        # IMPT (19 Feb) - once the drone detects a marker during the scan, it cannot return to doing the search path anymore! (for now)

        for wp in data['wp']:
            # Handle rotation if necessary. Scan for marker (on that side) after rotation, to see behind the drone's previous position
            dronecontroller.update_current_pos()
            dronecontroller.status = "Orienting"
            if wp['angle_deg'] != 0:
                if wp['angle_deg'] < 0:
                    dronecontroller.drone.rotate_clockwise(int(abs(wp['angle_deg'])))
                    dronecontroller.update_current_pos(rotation_deg=wp['angle_deg'])
                    scan_for_marker_oneside(dronecontroller, 'right')
                else:
                    dronecontroller.drone.rotate_counter_clockwise(int(abs(wp['angle_deg'])))
                    dronecontroller.update_current_pos(rotation_deg=wp['angle_deg'])
                    scan_for_marker_oneside(dronecontroller, 'left')
                time.sleep(1)  # Wait for rotation to complete
            
            # Handle forward movements in (20cm, 150cm] increments
            distance = wp['dist_cm']
            while distance > 200:
                dronecontroller.drone.move_forward(150)
                dronecontroller.update_current_pos(distance_cm=150)
                distance -= 150
                scan_for_marker_bothsides(dronecontroller)

            # After moving in 150cm increments, the remaining distance will be more than 50cm.
            dronecontroller.drone.move_forward(distance)
            dronecontroller.update_current_pos(distance_cm=distance)
            scan_for_marker_bothsides(dronecontroller)
    
    except Exception as e:
        logging.error(f"Error occurred during waypoint execution: {e}")     # this error will occur if waypoints not completed and detected something halfway?
        waypoints_success = False
    
    finally:
        # Stop any ongoing RC control commands
        dronecontroller.drone.send_rc_control(0, 0, 0, 0)
        if dronecontroller.drone.is_flying:
            if waypoints_success:
                logging.info("All waypoints completed. Initiating landing sequence...")
            else:
                logging.error("Waypoint execution did not complete successfully. Initiating emergency landing...")
            dronecontroller.drone.end()
        else:
            print("Drone already landed.")

def main():
    logger = setup_logging(params, "UnknownSearchArea")
    logger.info(f"Starting searcher server on drone {params.PI_ID} ...")
    
    try:
        # Initialize controller
        controller = DroneController(
            params.NETWORK_CONFIG, 
            drone_id=params.PI_ID, 
            laptop_only=params.LAPTOP_ONLY, 
            load_midas=False,
            imshow=params.IMSHOW
        )
    
        if not params.NO_FLY:
            controller.takeoff_simul([11,12,13])
            logging.info("Taking off for real...")
            controller.drone.go_to_height_PID(100)
            controller.drone.send_rc_control(0, 0, 0, 0)
        else:
            logging.info("Simulating takeoff. Drone will NOT fly.")
            
        time.sleep(2)
        
        # Execute waypoints
        execute_waypoints_scan_land(controller, params.WAYPOINTS_JSON)
        
        logging.info(f"Landing. Final Battery Level: {controller.drone.get_battery()}%")
        
    except KeyboardInterrupt:
        logging.info("Program interrupted by user")
    except Exception as e:
        logging.error(f"Error in main: {e}")
    finally:
        # Clean shutdown
        if controller:
            logging.info("Shutting down controller...")
            controller.shutdown()  # This will handle video stream and drone cleanup

if __name__ == "__main__":
    main()