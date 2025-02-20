# Tested 11 Feb - launch multiple clients

from swarmserverclient import MarkerClient
import time

marker_client1 = MarkerClient(drone_id=1)
marker_client2 = MarkerClient(drone_id=2)

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