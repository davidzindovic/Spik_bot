import pyrealsense2 as rs
import numpy as np
import cv2
import time
from typing import Tuple, Optional

# --- 1. CONFIGURATION ---
# Define the resolution and frame rate
WIDTH = 640
HEIGHT = 480
FPS = 30

# --- 2. VISION & TEXTURE FUNCTIONS ---

def detect_logs_by_texture(color_image: np.ndarray) -> np.ndarray:
    """
    Simulates log detection based on simple color/texture analysis.
    
    In a real scenario, this would use a robust method like:
    1. Color filtering (e.g., brown/tan range for bark)
    2. Local Binary Patterns (LBP) or Gabor Filters for texture features
    3. Contours/Shape detection (looking for circles/ovals)
    4. Machine Learning (e.g., a simple CNN or classic features + SVM)
    
    For this example, we'll use a simple brown color threshold.
    
    :param color_image: The RGB image frame from the camera.
    :return: A binary mask (white where a log is detected, black elsewhere).
    """
    # Convert to HSV color space for better color segmentation
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    
    # Simple threshold for a 'brown' bark color (H: Hue, S: Saturation, V: Value)
    # The lower/upper bounds need calibration for your specific logs and lighting.
    lower_brown = np.array([10, 50, 20])
    upper_brown = np.array([30, 255, 255])
    
    mask = cv2.inRange(hsv, lower_brown, upper_brown)
    
    # Optional: Clean up the mask using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    return mask

def find_contours_and_centers(mask: np.ndarray) -> list[Tuple[int, int]]:
    """Finds object contours and their image center points."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centers = []
    for contour in contours:
        # Filter small noise contours
        if cv2.contourArea(contour) < 500:
            continue
            
        # Calculate the center of the contour (Centroid)
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))
            
    return centers

# --- 3. POINT CLOUD & PEAK LOGIC FUNCTION ---

def get_peak_point_from_pointcloud(
    pc: rs.pointcloud, 
    depth_frame: rs.depth_frame, 
    color_frame: rs.video_frame,
    pixel_centers: list[Tuple[int, int]],
    depth_scale: float
) -> Optional[Tuple[float, float, float]]:
    """
    Finds the peak (closest point) of the closest log.
    
    :param pc: RealSense pointcloud object.
    :param depth_frame: The captured depth frame.
    :param color_frame: The captured color frame (for texture mapping).
    :param pixel_centers: List of (u, v) pixel coordinates for detected logs.
    :param depth_scale: The depth units to meters conversion factor.
    :return: (x, y, z) coordinates of the closest log's peak, or None.
    """
    
    # Compute the point cloud
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices()) # (N, 3) numpy array of (X, Y, Z)
    
    # 1. FIND THE CLOSEST LOG
    min_depth = float('inf')
    closest_log_3d_point: Optional[Tuple[float, float, float]] = None
    
    # For a simple solution, we'll check the depth at the center of each log
    # and find the log with the minimum Z (closest to camera).
    for u, v in pixel_centers:
        # Get depth value at pixel (u, v)
        depth_val = depth_frame.get_distance(u, v)
        
        if depth_val > 0.0 and depth_val < min_depth: # Ignore zero depth (invalid)
            min_depth = depth_val
            
            # Use the Depth frame's intrinsic to convert (u, v, Z) to (X, Y, Z)
            # This is simpler and more reliable than extracting a single point
            # from the full vtx array when using the pixel-based approach.
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            peak_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth_val)
            
            # peak_3d is [x, y, z] (m)
            closest_log_3d_point = (peak_3d[0], peak_3d[1], peak_3d[2])
            
    # 2. Refinement: Find the absolute closest point (peak) *within the closest log's area*
    # For simplicity, we assume the point found at the center (u, v) is a good approximation
    # of the peak for a roughly centered cylindrical object.
    # A true peak would involve segmenting the point cloud data for that log
    # and then finding the absolute minimum Z within that segment.
    
    return closest_log_3d_point

# --- 4. MAIN PROGRAM LOOP ---

def main():
    # Setup pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    
    # Start streaming
    profile = pipeline.start(config)
    
    # Get depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create align object to align depth to color frame
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create pointcloud object
    pc = rs.pointcloud()

    try:
        print("RealSense camera streaming started. Press 'q' to exit.")
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
                
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # --- VISION PROCESSING (Texture/Log Detection) ---
            log_mask = detect_logs_by_texture(color_image)
            log_centers_uv = find_contours_and_centers(log_mask)
            
            # Overlay detected centers for visualization
            display_image = color_image.copy()
            
            if log_centers_uv:
                # --- POINT CLOUD PROCESSING (Find Closest Peak) ---
                peak_3d_point = get_peak_point_from_pointcloud(
                    pc, depth_frame, color_frame, log_centers_uv, depth_scale
                )
                
                if peak_3d_point:
                    X_peak, Y_peak, Z_peak = peak_3d_point
                    
                    # Highlight the center of the closest log
                    # This is the (u, v) of the closest log, which we approximate as the peak
                    closest_log_center = log_centers_uv[0] # Simplification: assume first contour is the closest
                    cv2.circle(display_image, closest_log_center, 10, (0, 0, 255), -1) 
                    
                    # Print the required coordinates (X and Y relative to camera)
                    print(f"\nTarget Log Found (Closest):")
                    print(f"  X-Peak: {X_peak:.4f} m") # X (Horizontal) position
                    print(f"  Y-Peak: {Y_peak:.4f} m") # Y (Vertical) position
                    print(f"  Z-Depth: {Z_peak:.4f} m") # Z (Depth/Distance)
                    
                    # Display the coordinates on the image
                    text = f"Peak: X={X_peak:.2f}, Y={Y_peak:.2f}, Z={Z_peak:.2f} m"
                    cv2.putText(display_image, text, (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                else:
                    print("Log detected, but depth data is invalid.")
            else:
                print("No log texture detected.")
            
            # Display the result
            cv2.imshow('Log Detection and Peak Finder', display_image)
            cv2.imshow('Log Mask (Texture Filter)', log_mask)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # Make sure to install: pip install pyrealsense2 opencv-python numpy
    main()
