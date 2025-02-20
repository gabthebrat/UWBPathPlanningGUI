import importlib, sys, logging, os, time, cv2
import numpy as np
from typing import Optional

def load_params():
    """
    Loads a params.py file via the command line. Allows multiple drones to run on the same script.
    Example usage: 
        python -m UnknownArea_v2.main UnknownArea_v2.params2

    Returns:
        params: The module containing configuration parameters
    """
    default_params = "shared_params.params"
    if len(sys.argv) < 2:
        print(f"Load Params Usage: python script_name.py params_module. Trying default {default_params}.")  
        params_module = default_params
    else:
        params_module = sys.argv[1]

    try:
        params = importlib.import_module(params_module)
        print(f"Successfully loaded parameters from {params_module}!")
        print(f"Params are: {params}")      # For curiosity 18 Feb
        return params
    except ModuleNotFoundError:
        print(f"Error: Module {params_module} not found. Exiting script.")
        sys.exit(1)

def setup_logging(params: object, logger_name: Optional[str] = None) -> logging.Logger:
    """
    Sets up logging configuration based on parameters from params.py
    
    Args:
        params: Module containing logging configuration
        logger_name: Optional name for the logger. If None, uses default from params
    
    Returns:
        logging.Logger: Configured logger instance
    """
    # Get logging parameters
    config = params.LOGGING_CONFIG
    log_level = getattr(logging, config['level'].upper())
    
    # Create handlers
    file_handler = logging.FileHandler(config['filename'], mode='w')
    console_handler = logging.StreamHandler()
    
    # Create formatter
    formatter = logging.Formatter(config['format'])
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Configure root logger
    logging.basicConfig(
        level=log_level,
        handlers=[file_handler, console_handler]
    )
    
    # Create and configure specific logger
    logger_name = logger_name or config['default_logger_name']
    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    
    return logger

# TODO: move calib params into UnknownArea_v2 folder?
def get_calibration_parameters(TELLO_NO: str = 'E920EB_480P'): # BEST VERSION 30 JAN, V2.2 (to be tested)
    """
    Retrieves the camera matrix and distortion coefficients for the specified Tello drone.

    Args:
        TELLO_NO (str): Identifier for the Tello drone (e.g., 'D').

    Returns:
        tuple: Camera matrix and distortion coefficients.

    Raises:
        FileNotFoundError: If the calibration files for the specified Tello drone are not found.
    """
    # Construct file paths
    camera_matrix_path = os.path.join("calib_camera", f"camera_matrix_tello{TELLO_NO}.npy")
    dist_coeffs_path = os.path.join("calib_camera", f"dist_coeffs_tello{TELLO_NO}.npy")

    # Ensure files exist before loading
    if not os.path.exists(camera_matrix_path) or not os.path.exists(dist_coeffs_path):
        raise FileNotFoundError(f"Calibration files for Tello {TELLO_NO} not found.")

    # Load calibration parameters
    camera_matrix = np.load(camera_matrix_path)
    dist_coeffs = np.load(dist_coeffs_path)
    print(f"Calibration parameters obtained for {TELLO_NO}")
    return camera_matrix, dist_coeffs

def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle to the range [-180, 180). Useful for calculating Tello turn angle (e.g. for PID).
    """
    angle = angle % 360  # Ensure the angle is within [0, 360)
    if angle > 180:
        angle -= 360  # Shift down to [-180, 180)
    return angle

def capture_frame(frame_reader, max_retries:int = 3):
    # Get frame with retry mechanism
    retry_count = 0
    frame = None
    while frame is None and retry_count < max_retries:
        try:
            frame = frame_reader.frame
            if frame is None:
                logging.debug("Frame capture failed, retrying...")
                time.sleep(0.1)
                retry_count += 1
        except Exception as e:
            logging.warning(f"Frame capture failed: {e}. Returning empty frame.")
            frame = None
              
        if frame is None:
            logging.warning(f"Failed to capture frame after {retry_count} retries")

    return frame

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    logging.debug(f"ID {ids} found. Drawing axes.")
    marker_center = np.mean(corners[0], axis=0)
    cv2.circle(frame, 
                (int(marker_center[0]), int(marker_center[1])), 
                10, (0, 255, 0), -1)
    cv2.polylines(frame, 
                [corners[0].astype(np.int32)], 
                True, (0, 255, 0), 2)
    cv2.putText(frame, 
                f"ID: {ids}", 
                (int(marker_center[0]), int(marker_center[1] - 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
       
    cv2.drawFrameAxes(
        frame, CAMERA_MATRIX, DIST_COEFF, rvecs, tvecs, 10
    )
    
    x, y, z = tvecs[0]
    euclidean_distance = np.sqrt(x*x + y*y + z*z)
    distance_text = f"3D Distance to {ids[0]}: {euclidean_distance:.1f} cm"
    cv2.putText(frame, distance_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return frame
# Ensures variables are accessible by all scripts
params = load_params()
CAMERA_MATRIX, DIST_COEFF = get_calibration_parameters()
print("Calibration parameters obtained from shared_utils")