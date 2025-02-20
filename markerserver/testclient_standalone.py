# TESTING 20 FEB - to send UDP via client on phone etc. on the same network

import threading, socket, json, time
import logging
from typing import Dict, Set, Any, List, Literal

class MarkerClient:
    def __init__(self, drone_id=0, server_port=5005, broadcast_ip="255.255.255.255"):
        self.drone_id = drone_id
        self.server_port = server_port
        self.broadcast_ip = broadcast_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of address
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
        self.sock.bind(("0.0.0.0", 0))  # Bind to any available port
        self.ready = False
        self.takeoff_signal = False
        
        self.marker_status = {}  # Keeps a local copy of marker_status
        threading.Thread(target=self.receive_updates, daemon=True).start()

        self.send_update('marker', marker_id=-1)  # Send an initial message to register with the server
        logging.info(f"Marker client {drone_id} broadcasting on {self.broadcast_ip}:{self.server_port}")

    def send_takeoff_request(self, waiting_list:list):
        """
        Notifies the server that this drone is ready for takeoff and is waiting for the specified drones.
        """
        message = {
            "type": "takeoff_request",
            "drone_id": self.drone_id,
            "waiting_list": waiting_list,
            "ready": True
        }
        message_json = json.dumps(message).encode()

        for _ in range(5):  # Send the message 5 times for reliability
            self.sock.sendto(message_json, (self.broadcast_ip, self.server_port))
            time.sleep(0.03)  # Small delay to prevent flooding

        logging.info(f"Drone {self.drone_id} is ready and waiting for {waiting_list} to takeoff together.")
    
    def send_update(self, update_type:Literal["status","marker"], 
                    marker_id:int=None, detected:bool=None, landed:bool=None,
                    status_message:str=''):
        """
        19 Feb - keep separate from send_takeoff_request since it has an additional waiting_list argument

                message is a dictionary: 
                {"drone_id": 1, "update_type": "marker", "marker": 2, "detected": True}
                {"drone_id": 2, "update_type": "status", "status": "Centering"}

        """
        message:Dict = {"drone_id": self.drone_id}       # initializes the message with drone_id (client's class attribute)

        if update_type == "marker" and marker_id is not None:
            message["type"] = "marker"
            message["marker_id"] = marker_id
            if detected is not None:
                message["detected"] = detected
            if landed is not None:
                message["landed"] = landed

        elif update_type =="status":
            message["type"] = "status"
            message["status"] = status_message

        message_json = json.dumps(message).encode()
        for _ in range(5):  # Send the message 5 times for reliability
            self.sock.sendto(message_json, (self.broadcast_ip, self.server_port))
            time.sleep(0.03)  # Small delay to prevent flooding

        logging.debug(f"MarkerClient {self.drone_id} sent {message} (sent five times for reliability)")

    def receive_updates(self):  # Background thread
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                message = json.loads(data.decode())

                if message.get("type") == "takeoff" and self.drone_id in message.get("takeoff_list"):
                    logging.info(f"Received takeoff signal for Tello {self.drone_id}")
                    self.takeoff_signal = True  # Set flag for DroneController            

                else:
                    self.marker_status = message
                
                logging.debug(f"Received marker status from server: {self.marker_status}")
            except socket.timeout:  # Handle timeouts (if a timeout is set)
                pass
            except Exception as e:  # Catch any other exceptions
                logging.error(f"Error receiving updates: {e}")

    def is_marker_available(self, marker_id):
        marker_id = str(marker_id)  # Ensure it's a string to match dictionary keys
        marker_data:dict = self.marker_status.get(marker_id)
        if marker_data is None:
            return True  # Marker has never been seen before -> Available
        detected = marker_data.get("detected", False)   # False is the default value to return if "detected" key doesn't exist. If "detected": False, also returns false.
        landed = marker_data.get("landed", False)       # refer to TEST_markerstatusdict.py 
        return not (detected or landed)  # If either is True, it's NOT available. If no server, returns True by default (good for redundancy).

    def get_invalid_markers(self, markers_list: list) -> list:
        return [id for id in markers_list if not self.is_marker_available(id)]
    
    def cleanup(self):
        """Clean up the socket when the program exits."""
        if self.sock:
            self.sock.close()
            self.sock = None

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()

marker_client1 = MarkerClient(drone_id=1)
marker_client2 = MarkerClient(drone_id=2)

marker_client1.send_takeoff_request([1,2])
time.sleep(2)
marker_client2.send_takeoff_request([1,2])

marker_client1.send_update('status', status_message='takeoff')
marker_client2.send_update('status', status_message='takeoff')

try:
    while True:
        marker_client1.send_update('marker', 1, detected=True)
        time.sleep(1)
        marker_client1.send_update('marker', 1, detected=False)
        time.sleep(1)
        marker_client1.send_update('marker', 1, landed=True)
        time.sleep(1)
        marker_client2.send_update('marker', 2, detected=True)
        time.sleep(1)
        marker_client2.send_update('marker', 2, detected=False)
        time.sleep(1)
        marker_client1.send_update('status', status_message="Orienting")
        time.sleep(1)
        marker_client1.send_update('status', status_message="Moving Forward")
        time.sleep(1)

finally:
        marker_client1.send_update('status', status_message='landing')
        marker_client2.send_update('status', status_message='landing')