import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Enable IMU streams
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

# Start streaming
pipeline.start(config)

# Align depth to color
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Get IMU data
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (for visualization)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Display images
        cv2.imshow('Aligned Color Image', color_image)
        cv2.imshow('Aligned Depth Image', depth_colormap)

        # Display IMU data
        if accel_frame and gyro_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            print(f"Accel: {accel_data}, Gyro: {gyro_data}")

        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

