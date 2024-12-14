import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def main():
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

            # Extract masks and draw them on the images
            for result in results:
                if result is None:
                    continue
                masks = result.masks
                if masks is None:
                    continue

                for mask in masks.data:
                    # Convert mask to a format suitable for drawing
                    mask = mask.cpu().numpy().astype(np.uint8)

                    # Create a polygon from the mask
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.polylines(color_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.polylines(depth_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)

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
