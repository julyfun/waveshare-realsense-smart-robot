import pyrealsense2 as rs
import numpy as np
import cv2
import requests
from ultralytics import YOLO
from fast_math.kalman.linear_kalman import LinearKalmanAliveApi
from omegaconf import DictConfig
import hydra
from loguru import logger

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    # [rs]
    print('rs')
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_device(cfg.catch.dev)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)

    pc = rs.pointcloud()

    # [yolo]
    print('model')
    model = YOLO('yolov8l-seg.pt')

    # [kalman]
    kalman = LinearKalmanAliveApi(3, 2, 0.1, 10.0, 500.0)
    print('loop')

    try:
        while True:
            # [rs]
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data()) # mm -> m
            color_image = np.asanyarray(color_frame.get_data())
            img_time = aligned_depth_frame.get_timestamp() / 1e3 # ms -> s

            # [yolo]
            # [yolo.obj]
            obj_masks = []
            results = model(color_image)
            for res in results:
                if res is None:
                    continue
                masks = res.masks
                if masks is None:
                    continue
                for mask, class_id in zip(masks.data, res.boxes.cls):
                    mask = mask.cpu().numpy().astype(np.uint8)
                    label = res.names[int(class_id)]
                    if label in cfg.catch.labels:
                        area = np.sum(mask)
                        obj_masks.append((mask, area))

            if len(obj_masks):
                # [yolo.largest]
                # If kalman dead, find largest mask
                # If not dead, find one closest to current pos and no smaller than 0.6 * largest

                def get_mask_center_xyz(mask):
                    M = cv2.moments(mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX, cY = 0, 0
                    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                    depth = depth_image[cY, cX]
                    x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth)
                    return np.array([x, y, z]) / 1e3

                def fn():
                    largest_mask, largest_area = max(obj_masks, key=lambda x: x[1])
                    if kalman.dead(img_time):
                        return get_mask_center_xyz(largest_mask)
                    kalman_xyz = kalman.get_pos(img_time) # must not be None
                    closest_xyz = None
                    for m, a in obj_masks:
                        if a < cfg.catch.area_min_limit * largest_area:
                            continue
                        xyz = get_mask_center_xyz(m)
                        if closest_xyz is None or np.linalg.norm(xyz - kalman_xyz) < np.linalg.norm(closest_xyz - kalman_xyz):
                            closest_xyz = xyz
                    return closest_xyz

                so_obs = fn()

                kalman.update(img_time, so_obs)
                so_xyz = kalman.get_pos(img_time)

                try:
                    def fn():
                        url = "http://localhost:4060/pub-tf2"
                        data = {
                            "frame_id": "rs_arm",
                            "child_frame_id": "obj",
                            "xyz": so_xyz.tolist(),
                            "xyzw": [0.0, 0.0, 0.0, 1.0]
                        }

                        response = requests.post(url, json=data)

                    fn()

                    def fn(tar, src):
                        url = "http://localhost:4060/get-tf2"
                        data = {
                            "frame_id": tar,
                            "child_frame_id": src,
                        }

                        response = requests.post(url, json=data)
                        data = response.json()
                        if data["status"] == "success":
                            xyz = data["xyz"]
                            xyzw = data["xyzw"]
                            print("Translation (xyz):", xyz, type(xyz))
                            return xyz, xyzw
                        else:
                            print("Error:", data["message"])

                    obj_xyz, _ = fn("base_link", "obj")
                    logger.info(f'z: {obj_xyz[2]}')

                    def fn():
                        url = "http://localhost:4060/move-v2"
                        data = {
                            "x": 0.0,
                            "y": -0.2,
                            "z": float(np.clip(obj_xyz[2], 0.05, 0.35)),
                        }
                        response = requests.post(url, json=data)
                        data = response.json()
                        print(data)

                    fn()
                except Exception as e:
                    logger.error(e)

            # [vis]
            for res in results:
                if res is None:
                    continue
                masks = res.masks
                if masks is None:
                    continue
                for mask, class_id in zip(masks.data, res.boxes.cls):
                    mask = mask.cpu().numpy().astype(np.uint8)
                    label = res.names[int(class_id)]
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.polylines(color_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.polylines(depth_image, contours, isClosed=True, color=(0, 255, 0), thickness=2)

                    # Find the centroid of the mask to place the label
                    M = cv2.moments(mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.putText(color_image, label, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Aligned RGB and Depth', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()