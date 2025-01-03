import pyrealsense2 as rs
import requests
import numpy as np
import cv2
from ultralytics import YOLO

def main():
    # Configure depth and color streams
    print("Configuring streams...")
    pipeline = rs.pipeline()
    config = rs.config()
    print("config: ", config)

    # Enable depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Load YOLOv8 segmentation model
    print("Loading model...")
    model = YOLO('yolov8l-seg.pt')  # Ensure you have the correct model file
    print("Model loaded")

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

                for mask, class_id in zip(masks.data, result.boxes.cls):
                    # Convert mask to a format suitable for drawing
                    mask = mask.cpu().numpy().astype(np.uint8)

                    # Create a polygon from the mask
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.polylines(color_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)

                    # Find the centroid of the mask to place the label
                    M = cv2.moments(mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        label = result.names[int(class_id)]
                        cv2.putText(color_image, label, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            if largest_mask is not None:
                # Calculate the average depth within the largest mask
                masked_depth = np.where(largest_mask, depth_image, 0)
                non_zero_depths = masked_depth[masked_depth > 0]
                if non_zero_depths.size > 0:
                    average_depth = np.mean(non_zero_depths)
                    print(f"Average Depth of Largest Human: {average_depth:.2f} mm")

                # Create a polygon from the largest mask
                contours, _ = cv2.findContours(largest_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.polylines(color_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.polylines(depth_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)

            # [depth]
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Aligned RGB and Depth', images)

            # [发送 POST 请求]
            # data = {
            #     "flts": [1.1, 2.2, 3.3, 4.4]  # 假设这是你要发送的浮点数
            # }
            # response = requests.post("http://127.0.0.1:4050/ttf/", json=data)
            # print(response.json())

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
