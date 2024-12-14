import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import math

def main():
    print('loading rs pipeline')
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Load YOLOv8 segmentation model
    model = YOLO('yolov8l-seg.pt')  # Ensure you have the correct model file

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Perform segmentation on the color image
            results = model(color_image)

            largest_mask = None
            largest_area = 0

            # Extract masks and find the largest human mask
            for result in results:
                if result is None:
                    continue
                masks = result.masks
                if masks is None:
                    continue

                for mask in masks.data:
                    # Convert mask to a format suitable for drawing
                    mask = mask.cpu().numpy().astype(np.uint8)

                    # Calculate the area of the mask
                    area = np.sum(mask)

                    # Check if this is the largest mask
                    if area > largest_area:
                        largest_area = area
                        largest_mask = mask

            if largest_mask is not None:
                # Calculate the average depth and position within the largest mask
                masked_depth = np.where(largest_mask, depth_image, 0)
                non_zero_depths = masked_depth[masked_depth > 0]
                if non_zero_depths.size > 0:
                    average_depth = np.mean(non_zero_depths)

                    # Calculate centroid of the largest mask
                    M = cv2.moments(largest_mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0

                    # Get depth intrinsics
                    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

                    # Convert pixel coordinates to 3D coordinates
                    x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], average_depth)

                    # Compute yaw and pitch
                    yaw = -math.degrees(math.atan2(x, z))
                    pitch = -math.degrees(math.atan2(y, z))

                    print(f"Average 3D Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    print(f"Yaw: {yaw:.2f} degrees, Pitch: {pitch:.2f} degrees")

                # Create a polygon from the largest mask
                contours, _ = cv2.findContours(largest_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.polylines(color_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Aligned RGB and Depth', images)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
