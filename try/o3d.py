import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

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

# Create an Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window('Open3D Point Cloud')

# Create a point cloud object
point_cloud = o3d.geometry.PointCloud()

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

        # Get intrinsic parameters of the depth frame
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # Create Open3D RGBD image
        depth_o3d = o3d.geometry.Image(depth_image)
        color_o3d = o3d.geometry.Image(color_image)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, convert_rgb_to_intensity=False)

        # Generate point cloud
        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                depth_intrinsics.width, depth_intrinsics.height,
                depth_intrinsics.fx, depth_intrinsics.fy,
                depth_intrinsics.ppx, depth_intrinsics.ppy))

        # Flip it, otherwise the point cloud will be upside down
        point_cloud.transform([[1, 0, 0, 0],
                               [0, -1, 0, 0],
                               [0, 0, -1, 0],
                               [0, 0, 0, 1]])

        # Visualize the point cloud
        vis.clear_geometries()
        vis.add_geometry(point_cloud)
        vis.poll_events()
        vis.update_renderer()

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
    vis.destroy_window()
    cv2.destroyAllWindows()
