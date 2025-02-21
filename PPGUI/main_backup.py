# Still Testing and Verifying Orientations and Errors List

import pygame
import json
import math
from constants import *
from config import *
from utils import *

import tkinter as tk
from tkinter import simpledialog

from typing import List, Tuple
import datetime

"""
UAV Lab Test Area: 7m x 6m (700cm x 600cm)
Nanyang Audi Test Area: 2000cm x 2000cm,
Original image: 708px x 708px

Import Hierarchy: (Ensure imports follow a unidirectional structure. Constants should flow downstream to avoid circular dependencies.)
    config --> constants --> utils --> main

IMPT: To toggle saving last waypoint - see "save_json" function!

pygame origin: top left
uwb origin: bottom left

TODO:
- Resizeable
- Load new markers json (uwb_trace.json)

"""

pygame.init()
# screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT], pygame.RESIZABLE)
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])
screen.fill(WHITE)
pygame.display.set_caption("Drone Path Planner (Test Area)")

root = tk.Tk()  # Create a Tkinter root window
root.withdraw()  # Hide the main Tkinter window
user_input = ""  # Store the user's input

def find_intermediate_waypoints(start, end, interval_cm=50):     # includes both start and end points
    """
    Returns intermediate waypoints along a line from 'start' to 'end' at each specified interval (in cm).
    """
    intermediate_waypoints = [start]
    dist_cm, _ = get_dist_btw_pos_px(start, end)
    
    if dist_cm < interval_cm:
        return intermediate_waypoints + [end]  # Just add the endpoint if the segment is less than the interval

    # Calculate direction vector
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dist_px = math.hypot(dx, dy)
    step_px = interval_cm / MAP_SIZE_COEFF
    
    # Normalize direction vector to the interval length
    step_x = dx * (step_px / dist_px)
    step_y = dy * (step_px / dist_px)

    # Add waypoints at each interval along the line
    current_pos = start
    while True:
        # Calculate next position by moving in the direction of the line
        next_pos = (int(current_pos[0] + step_x), int(current_pos[1] + step_y))
        
        # Stop if the next waypoint would overshoot the endpoint
        remaining_dist, _ = get_dist_btw_pos_px(next_pos, end)
        if remaining_dist < interval_cm:
            break
            
        intermediate_waypoints.append(next_pos)
        current_pos = next_pos

    intermediate_waypoints.append(end)  # Ensure the final endpoint is included
    return intermediate_waypoints

def draw_test_boundaries():
    """
    Draw test area boundaries and information
    """
    # Draw border
    pygame.draw.rect(screen, BLUE, (0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), 2)
    
    # Draw Take-off Area (scaled from 10m x 4m to test area)
    takeoff_width = int(SCREEN_WIDTH * 0.5)  # Using half width for take-off area
    takeoff_height = int(SCREEN_HEIGHT * 0.2)  # Using 20% of height
    takeoff_x = (SCREEN_WIDTH - takeoff_width) // 2
    takeoff_y = SCREEN_HEIGHT - takeoff_height
    pygame.draw.rect(screen, GREEN, (takeoff_x, takeoff_y, takeoff_width, takeoff_height), 2)
    

def draw_grid():
    """
    Draw a grid with 1m spacing
    """
    # Calculate pixel spacing for 1m grid
    grid_spacing_px = int(100 / MAP_SIZE_COEFF)  # 100cm = 1m
    
    # Draw vertical lines
    for x in range(0, SCREEN_WIDTH, grid_spacing_px):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, SCREEN_HEIGHT), 1)
    
    # Draw horizontal lines
    for y in range(0, SCREEN_HEIGHT, grid_spacing_px):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (SCREEN_WIDTH, y), 1)

def draw_text_overlay():
    """
    Draw text information overlay
    """
    font = pygame.font.Font(None, 24)
    
    # Draw info texts
    text_surface = font.render(f"Test Area: {ACTUAL_WIDTH/100}m x {ACTUAL_HEIGHT/100}m", True, BLUE)
    screen.blit(text_surface, (10, SCREEN_HEIGHT-50))
    
    text_info = font.render("S: save and close. | O: orthogonal mode. | I: intermediate mode", True, BLUE)
    screen.blit(text_info, (10, SCREEN_HEIGHT-35))

    text_info = font.render("CCW is +ve. Assumes drone starts heading 180.", True, BLUE)
    screen.blit(text_info, (10, SCREEN_HEIGHT-20))

def load_json_waypoints(filename):
    """
    Load waypoints from a JSON file and return the pixel coordinates.
    NOTE: Does not load last waypoint unless it is provided (i.e. distance,angle = 0,0)
    
    Args:
        filename (str): Name of the JSON file to load (without .json extension)
    
    Returns:
        list: Pixel coordinates of waypoints
    """
    global path_wp, path_wp_cm, action_index
    
    try:
        with open(f'{filename}.json', 'r') as f:
            data = json.load(f)
        
        # Reset existing waypoints
        path_wp = []
        path_wp_cm = []
        action_index = 0
        
        screen_setup(screen)  # Refresh the screen

        # Iterate through waypoints and convert back to pixel coordinates
        for waypoint in data['wp']:
            x_cm = waypoint['position_cm']['x']
            y_cm = waypoint['position_cm']['y']
            
            # Convert cm to pixel coordinates
            x_px = int(x_cm / MAP_SIZE_COEFF)
            y_px = int(SCREEN_HEIGHT - y_cm / MAP_SIZE_COEFF)
            
            # Process each waypoint
            process_waypoint_input((x_px, y_px))
        
        print(f"Successfully loaded waypoints from {filename}.json")
        return path_wp
    
    except FileNotFoundError:
        print(f"Error: File {filename}.json not found.")
        return None


def load_marked_positions(filename='UWBViz/uwb_trace'):
    """
    Load marked positions of obstacles and walls etc. from JSON file and draw them on screen.
    Can load from multiple files
    Press 'Z' to reset - how convenient!
    
    Args:
        filename (str): Name of the JSON file to load
    """

    global static_screen_snapshot
    
    try:
        with open(f"{filename}.json", 'r') as f:
            marked_positions = json.load(f)["points"] 
            walls = []
            current_wall_start = None

            # Process walls from wall_start to wall_end
            for point in marked_positions:
                if point["type"] == "wall_start":
                    current_wall_start = (point["x"], point["y"])
                elif point["type"] == "wall_end" and current_wall_start:
                    walls.append((current_wall_start, (point["x"], point["y"])))
                    current_wall_start = None  # Reset for the next segment

            # Draw walls as thick black lines
            for wall in walls:
                start_x, start_y = wall[0][0]*100, wall[0][1]*100
                end_x, end_y = wall[1][0]*100, wall[1][1]*100
                pygame.draw.line(screen, (100, 100, 200), (start_x/MAP_SIZE_COEFF, SCREEN_HEIGHT - start_y/MAP_SIZE_COEFF), (end_x/MAP_SIZE_COEFF, SCREEN_HEIGHT - end_y/MAP_SIZE_COEFF), 5)

            # Draw victims as green triangles
            for victim in (p for p in marked_positions if p["type"] == "victim"):
                screen_x, screen_y = victim["x"]*100, victim["y"]*100
                pygame.draw.polygon(screen, (0, 255, 0), [
                    (screen_x/MAP_SIZE_COEFF, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF - 10),
                    (screen_x/MAP_SIZE_COEFF - 10, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF + 10),
                    (screen_x/MAP_SIZE_COEFF + 10, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF + 10)
                ])

            # Draw dangers as red triangles
            for danger in (p for p in marked_positions if p["type"] == "danger"):
                screen_x, screen_y = danger["x"]*100, danger["y"]*100
                pygame.draw.polygon(screen, (255, 0, 0), [
                    (screen_x/MAP_SIZE_COEFF, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF - 10),
                    (screen_x/MAP_SIZE_COEFF - 10, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF + 10),
                    (screen_x/MAP_SIZE_COEFF + 10, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF + 10)
                ])

            # Draw pillars as black circles
            for pillar in (p for p in marked_positions if p["type"] == "pillar"):
                screen_x, screen_y = pillar["x"]*100, pillar["y"]*100
                pygame.draw.circle(screen, (0, 0, 0), (screen_x/MAP_SIZE_COEFF, SCREEN_HEIGHT - screen_y/MAP_SIZE_COEFF), 10, 2)

            static_screen_snapshot = screen.copy()      # IMPT: This is the line that sets the background!
        print("[INFO] Marked positions loaded successfully.")
    except FileNotFoundError:
        print("[INFO] No saved marked positions found.")
    except Exception as e:
        print(f"[ERROR] Could not load marked positions: {e}")

def process_waypoint_input(pos):

    """
    Takes in mouse coordinates pos and:
    1) Saves coordinates into path_wp (for GUI) and path_wp_cm (for export). 
    2) Draws waypoint marker and labels coordinate (in cm)
    3) Also calculates intermediate waypoints if required.
       NOTE 6 Jan: May not need INT_MODE function! Since PPFLY already divides long waypoints into shorter chunks.
    """

    global action_index, path_wp, static_screen_snapshot

    pos_cm = [round(pos[0]*MAP_SIZE_COEFF), round(ACTUAL_HEIGHT-pos[1]*MAP_SIZE_COEFF)]   # IMPT: Changes 0,0 from top left to bottom left

    # TODO 7 JAN - how to prevent multiple recurring waypoints at the exact same spot (especially if pressing STARTPOS_n)
    
    path_wp.append(pos)
    path_wp_cm.append(pos_cm)
    print(f"\nWaypoint {action_index}: ({pos_cm[0]}cm, {pos_cm[1]}cm)")

    if action_index > 0:
        label_distance(path_wp, pos)
        if INT_MODE:    # Find and draw intermediate points
            intermediate_points = find_intermediate_waypoints(path_wp[-2], pos, interval_cm=INTERMEDIATE_DIST)
            path_wp = path_wp[0:-1] + intermediate_points[1:-1] + [path_wp[-1]]
            print(f"Processing Intermediate Waypoints: {intermediate_points}")
            for i, point in enumerate(intermediate_points):
                pygame.draw.circle(screen, BLUE, point, 5)
                if i > 0:
                    pygame.draw.line(screen, RED, intermediate_points[i - 1], point, 2)
                    dist_cm, _ = get_dist_btw_pos_px(intermediate_points[i - 1], point)
                    print(f"Distance to previous waypoint: {dist_cm}cm")
                    
                    if dist_cm > 500:
                        print("[WARNING] Movement exceeds Tello's 500cm limit!")
        else:       # Only considers endpoints, without intermediates
            pygame.draw.circle(screen, BLUE, pos, 5)
            pygame.draw.line(screen, RED, path_wp[-2], path_wp[-1], 2)
            dist_cm, _ = get_dist_btw_pos_px(path_wp[-2], path_wp[-1])
            print(f"Distance to previous waypoint: {dist_cm}cm")      
            if dist_cm > 500:
                print("[WARNING] Movement exceeds Tello's 500cm limit!")

    
    # Draw waypoint marker
    pygame.draw.circle(screen, BLUE, pos, 5)
    font = pygame.font.Font(None, 24)

    label_text = font.render(f"{action_index}: {pos_cm}", True, (0, 0, 0))
    text_rect = label_text.get_rect(center=(pos[0]+35, pos[1]+9))
    bg_rect = pygame.Rect(text_rect.left + 10, text_rect.top - 2, text_rect.width + 10, text_rect.height + 5)

    pygame.draw.rect(screen, GREY, bg_rect)
    screen.blit(label_text, pos)
    static_screen_snapshot = screen.copy()
    action_index += 1
    return

def label_distance(path_wp, current_pos):
    # Obtains current distance from previous waypoint
    # Calculate and display the distance in cm on the preview line
    font = pygame.font.Font(None, 24)
    dist_cm, _ = get_dist_btw_pos_px(path_wp[-1], current_pos)
    midpoint_x = (path_wp[-1][0] + current_pos[0]) // 2
    midpoint_y = (path_wp[-1][1] + current_pos[1]) // 2
    distance_text = font.render(f"{dist_cm} cm", True, (0, 0, 0))

    # Get the size and position of the text
    text_rect = distance_text.get_rect(center=(midpoint_x, midpoint_y))

    # Create the background rectangle based on the text_rect and position it at the center of the text
    bg_rect = pygame.Rect(text_rect.left - 2, text_rect.top - 2, text_rect.width + 4, text_rect.height + 4)

    # Draw the white background rectangle at the center and then blit the text on top
    if dist_cm != 0:
        pygame.draw.rect(screen, WHITE, bg_rect)
        screen.blit(distance_text, text_rect)

    return

def save_json(path_wp, json_name:str):     # Computes and saves final waypoints into .json file
    """
    :Args:
    json_name: Do NOT include .json extension

    Format of json:
    - distance and angle to NEXT waypoint
    - current waypoint
    i.e. by default, last waypoint is not stored (CAN BE TOGGLED BY uncommenting under "else")
    """
    if len(path_wp) > 1:      
        path_dist_cm = []
        path_dist_px = []
        path_angle = []       
        for index in range(len(path_wp)):
            if index < (len(path_wp) - 1):
                dist_cm, dist_px = get_dist_btw_pos_px(path_wp[index], path_wp[index+1])
                path_dist_cm.append(dist_cm)
                path_dist_px.append(dist_px)

                if index == 0:      ## IMPT: This part assumes the drone starts with initial heading = 180
                    dummy_point = (path_wp[0][0],path_wp[0][1]-10)  # TODO 6 Jan: Can remove this? MAY NOT WORK PROPERLY YET
                    angle = get_yaw_angle(dummy_point, path_wp[index+1], path_wp[index]) - INITIAL_HEADING
                    if angle > 180:
                        angle -= 360
                    elif angle < -180:
                        angle += 360
                else:
                    angle = get_yaw_angle(path_wp[index-1], path_wp[index+1], path_wp[index])
        
                print(f"{index} yaw angle = {round(angle,0)}")    # TODO 30/10 Check whether can fly with decimal yaw commands
                path_angle.append(round(angle,0))

            else: # NEW 7 JAN - final waypoint with remaining distance = 0, angle = 0 (IMPT: ENABLE OR DISABLE THIS AS NEEDED. Alternatively, just delete the last waypoint in the json.)
                  # TBC 23 Jan : is this really needed for UWB localization?
                path_dist_cm.append(0)
                path_dist_px.append(0)
                path_angle.append(0)

        # Create waypoints data
        waypoints = []
        for index in range(len(path_dist_cm)):
            position_cm_x = int(path_wp_cm[index][0])
            position_cm_y = int(path_wp_cm[index][1])
            
            waypoints.append({
                "dist_cm": path_dist_cm[index],
                "dist_px": path_dist_px[index],
                "angle_deg": path_angle[index],
                "position_cm": {
                    "x": position_cm_x,
                    "y": position_cm_y,
                }
            })


        now = datetime.datetime.now()
        formatted_datetime = now.strftime("%d %m %H:%M")

        # Save to JSON file
        with open(f'{json_name}.json', 'w+') as f:
            json.dump({
                "README": "JSON stores current waypoint and distance + angle to NEXT waypoint i.e. final waypoint is when dist_cm = angle_deg = 0!",
                "wp": waypoints,
                "test_area": {  # EXTRA; not being used caa 7 JAN
                    "width_cm": ACTUAL_WIDTH,
                    "height_cm": ACTUAL_HEIGHT,
                    "scale_factor": MAP_SIZE_COEFF},
                "competition_notice": "This path was planned in a test area. Scale adjustments needed for 20m x 20m competition field.",
                "datetime": formatted_datetime
            }, f, indent=4)
        print(f"json saved as {json_name}.json")
        print("\n------------------\npath_wp:", path_wp, "\npath_wp_cm:",path_wp_cm,"\n------------------") 

    else:
        print("No waypoints saved.")

# Main program
running = True
processing_input = False
editing = True

def screen_setup(screen):
    global static_screen_snapshot
    bground = Background(BGPIC, [0, 0])  # Changed from 'image.png' to 'field.png'
    screen.blit(bground.image, bground.rect)
    draw_grid()
    draw_test_boundaries()
    draw_text_overlay()
    static_screen_snapshot = screen.copy()

screen_setup(screen)

path_wp: List[Tuple[float, float]] = []
"""Stores waypoints as pixel coordinates; (0,0) is top left of screen"""
path_wp_cm: List[Tuple[float, float]] = []
"""Stores waypoints as cm coordinates; (0,0) is bottom left of screen"""

action_index = 0
last_pos = None

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and editing == True:
            pos = pygame.mouse.get_pos()
            
            # In Orthogonal Mode, restrict movement to either the X or Y axis
            if ORT_MODE and action_index > 0:
                last_wp = path_wp[-1]
                if abs(pos[0] - last_wp[0]) > abs(pos[1] - last_wp[1]):
                    pos = (pos[0], last_wp[1])  # Lock movement along X-axis
                else:
                    pos = (last_wp[0], pos[1])  # Lock movement along Y-axis
            process_waypoint_input(pos)   

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_o:  # Toggle Orthogonal Mode with 'O' key
                ORT_MODE = not ORT_MODE
                mode_status = "ON" if ORT_MODE else "OFF"
                print(f"Orthogonal Mode is now {mode_status}")

            # elif event.key == pygame.K_i:  # Toggle Intermediate Mode with 'I' key NOTE: DISABLED 6 JAN - pp_fly has this function already
            #     INT_MODE = not INT_MODE
            #     mode_status = "ON" if INT_MODE else "OFF"
            #     print(f"Intermediate Mode is now {mode_status}")

            elif event.key == pygame.K_e: 
                process_waypoint_input(ENDPOS)

            elif event.key == pygame.K_1: # THIS IS TO START/END AT THE SAME POS
                process_waypoint_input(STARTPOS_1)

            elif event.key == pygame.K_2: # THIS IS TO START/END AT THE SAME POS
                process_waypoint_input(STARTPOS_2)

            elif event.key == pygame.K_3: # THIS IS TO START/END AT THE SAME POS
                process_waypoint_input(STARTPOS_3)

            elif event.key == pygame.K_4: # THIS IS TO START/END AT THE SAME POS
                process_waypoint_input(STARTPOS_4)

            elif event.key == pygame.K_s:
                running = False
                user_input = simpledialog.askstring("Save Waypoints", 
                                                        f"Enter JSON filename to save (without .json extension) \nDefault: {WAYPOINTS_JSON_DEFAULT}")
                filename = WAYPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}"
                save_json(path_wp, filename)
                print(f"Exited. Saved as {filename}.json.")
                print_json_waypoints(filename)

            elif event.key == 27:   # esc key
                running = False
                print("Exited without saving")
    
            elif event.key == 13:   # enter key
                editing = not editing
                if editing:
                    print("\nEditing resumed.")
                else:
                    print("\nEditing paused. Press Enter to resume.")

            elif event.key == pygame.K_r:   # RESET
                path_wp = []
                action_index = 0
                last_pos = None
                print("\nScreen reset completed. Waypoints cleared.")
                screen_setup(screen)

            elif event.key == pygame.K_z:   # TODO: Undo doesn't work with intermediate points! Might need to use a separate list of intermediate points
                if path_wp:  # Check if there are waypoints to undo
                    screen_setup(screen)  # Clear the screen to start fresh
                    path_wp_temp = path_wp[:-1]  # Remove the last waypoint
                    path_wp = []
                    action_index = 0
                    if path_wp:
                        last_pos = path_wp[-1]  # Set last_pos to the new last waypoint
                    else:
                        last_pos = None  # No waypoints left

                    # Redraw remaining waypoints and lines
                    for i in range(len(path_wp_temp)):
                        process_waypoint_input(path_wp_temp[i])
                        # print(f"i is {i}, pathwp is{path_wp_temp}, currentpos is {path_wp_temp[i]}")

            elif event.key == pygame.K_l:  # 'L' key to load JSON waypoints
                user_input = simpledialog.askstring("Load Waypoints", 
                                                        f"Enter JSON filename to load (without .json extension) \nDefault: {WAYPOINTS_JSON_DEFAULT}")
                filename = WAYPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}"
                load_json_waypoints(filename)

            elif event.key == pygame.K_m:  # 'M' key to load markers    # NEEDS WORK 15 FEB
                user_input = simpledialog.askstring("Load Waypoints", 
                                                        f"Enter JSON filename to load (without .json extension) \nDefault: {MARKEDPOINTS_JSON_DEFAULT}")
                filename = MARKEDPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}"
                print(filename)
                load_marked_positions(filename)

        # if event.type == pygame.VIDEORESIZE:  # Handle window resize events
        #     SCREEN_WIDTH = event.w  # Update width and height
        #     SCREEN_HEIGHT = event.h
        #     screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)  # Recreate the screen surface
        #     # Important: Redraw everything after resizing, as the surface is new.
        #     screen_setup(screen)

        #     # Iterate through waypoints and convert back to pixel coordinates
        #     for waypoint in path_wp:
        #         x_px = int(waypoint[0] / MAP_SIZE_COEFF)
        #         y_px = int(SCREEN_HEIGHT - waypoint[1] / MAP_SIZE_COEFF)
                
        #         # Process each waypoint
        #         process_waypoint_input((x_px, y_px))
    
    # Display the current mouse position at the top left
    current_pos = pygame.mouse.get_pos()


    # Update line preview AND overlay distance in cm at the line midpoint
    if current_pos != last_pos and len(path_wp) > 0 and not processing_input and editing:
        screen.blit(static_screen_snapshot, (0, 0))

        if ORT_MODE:
            last_wp = path_wp[-1]
            if abs(current_pos[0] - last_wp[0]) > abs(current_pos[1] - last_wp[1]):
                current_pos = (current_pos[0], last_wp[1])  # Constrain to X-axis
            else:
                current_pos = (last_wp[0], current_pos[1])  # Constrain to Y-axis

        pygame.draw.line(screen, ORANGE, path_wp[-1], current_pos, 2)
        last_pos = current_pos
        label_distance(path_wp, current_pos)

    # Display the current mouse position at the top left
    pygame.draw.rect(screen, (0, 0, 0), (0, 0, 100, 30))
    font = pygame.font.Font(None, 24)
    text = font.render(f"({round(current_pos[0]*MAP_SIZE_COEFF)}, {round(ACTUAL_HEIGHT-current_pos[1]*MAP_SIZE_COEFF)})", True, (255, 255, 255))
    screen.blit(text, (10, 10))    
    processing_input = False
    pygame.display.update()

pygame.quit()

