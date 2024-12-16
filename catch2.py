import pyrealsense2 as rs
import numpy as np
import cv2
import requests
from ultralytics import YOLO
from fast_math.kalman.linear_kalman import LinearKalmanAliveApi
from omegaconf import DictConfig
import hydra
import time
from loguru import logger

class CatchSystem:
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg

        # [rs]
        print('rs')
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # self.config.enable_device(cfg.catch.dev)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.pc = rs.pointcloud()

        # [yolo]
        print('model')
        self.model = YOLO('yolov8m-seg.pt')

        # [kalman]
        self.kalman = LinearKalmanAliveApi(3, 2, 0.1, 10.0, 5.0)
        print('loop')

        # [catch_test]
        self.catch_test_last_move = -10086.666

        # [cache]
        self.cache_cpst_z = None
        self.state_grip = "close"

    def get_mask_center_xyz(self, mask, depth_image, aligned_depth_frame):
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

    def run(self):
        try:
            while True:
                # [rs]
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
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
                results = self.model(color_image)
                for res in results:
                    if res is None:
                        continue
                    masks = res.masks
                    if masks is None:
                        continue
                    for mask, class_id in zip(masks.data, res.boxes.cls):
                        mask = mask.cpu().numpy().astype(np.uint8)
                        label = res.names[int(class_id)]
                        if label in self.cfg.catch.labels:
                            area = np.sum(mask)
                            obj_masks.append((mask, area))

                if len(obj_masks):
                    # [yolo.largest]
                    # If kalman dead, find largest mask
                    # If not dead, find one closest to current pos and no smaller than 0.6 * largest

                    def fn():
                        largest_mask, largest_area = max(obj_masks, key=lambda x: x[1])
                        if self.kalman.dead(img_time):
                            return self.get_mask_center_xyz(largest_mask, depth_image, aligned_depth_frame)
                        kalman_xyz = self.kalman.get_pos(img_time) # must not be None
                        closest_xyz = None
                        for m, a in obj_masks:
                            if a < self.cfg.catch.area_min_limit * largest_area:
                                continue
                            xyz = self.get_mask_center_xyz(m, depth_image, aligned_depth_frame)
                            if closest_xyz is None or np.linalg.norm(xyz - kalman_xyz) < np.linalg.norm(closest_xyz - kalman_xyz):
                                closest_xyz = xyz
                        return closest_xyz

                    so_obs = fn()

                    # not used
                    self.kalman.update(img_time, so_obs)
                    so_xyz = self.kalman.get_pos(img_time)

                    try:
                        print(so_obs)
                        def fn():
                            url = "http://localhost:4060/pub-tf2"
                            data = {
                                "frame_id": "rs_arm",
                                "child_frame_id": "obj",
                                "xyz": so_obs.tolist(),
                                "xyzw": [0.0, 0.0, 0.0, 1.0]
                            }

                            response = requests.post(url, json=data)

                        fn()

                        if self.kalman.dead(img_time):
                            raise Exception("Kalman dead")

                        def tf2(tar, src):
                            url = "http://localhost:4060/get-tf2"
                            data = {
                                "frame_id": tar,
                                "child_frame_id": src,
                            }

                            response = requests.post(url, json=data)
                            data = response.json()
                            if data["status"] == "success":
                                xyz = np.array(data["xyz"])
                                xyzw = np.array(data["xyzw"])
                                return xyz, xyzw
                            else:
                                print("Error:", data["message"])
                                return None

                        if self.cache_cpst_z is None:
                            self.cache_cpst_z = tf2("base_link", "link2")[0][2]
                        obj_xyz_in_link2 = tf2("base_link", "obj")[0] - np.array([0.0, 0.0, self.cache_cpst_z])
                        logger.info(f'z: {obj_xyz_in_link2[2]}')

                        def grip(x):
                            url = "http://localhost:4060/grip"
                            data = {
                                "data": x,
                            }
                            response = requests.post(url, json=data)

                        def fn():
                            cp = self.cfg.catch.compensate
                            xyz_cp = obj_xyz_in_link2 + np.array([cp.x, cp.y, cp.z])

                            xy_proj = xyz_cp[:2] # tmp
                            exp_norm = np.clip(np.linalg.norm(xy_proj) - 0.03, 0.1, 0.50) # tmp
                            so_xy = xy_proj / np.linalg.norm(xy_proj) * exp_norm

                            send_xyz = np.array([so_xy[0], so_xy[1], np.clip(xyz_cp[2], 0.0, 0.35)])
                            print(f'send_xyz: {send_xyz}')

                            cur_xyz_in_link2 = tf2("base_link", "hand_tcp")[0] - np.array([0.0, 0.0, self.cache_cpst_z])
                            if np.linalg.norm(cur_xyz_in_link2 - xyz_cp) < 0.10:
                                # grip
                                if self.state_grip == "close":
                                    grip(3.0)
                                    self.state_grip = "open"
                            else:
                                if self.state_grip == "open":
                                    grip(0.0)
                                    self.state_grip = "close"

                            url = "http://localhost:4060/move"
                            data = {
                                "x": send_xyz[0],
                                "y": send_xyz[1],
                                "z": send_xyz[2],
                            }
                            # this service gonna make base_link to hand_tcp to (x, y, z + cpst_z)
                            response = requests.post(url, json=data)


                            data = response.json()
                            print(data)

                        if img_time - self.catch_test_last_move > self.cfg.catch.move_time_interval:
                            fn()
                            self.catch_test_last_move = img_time

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
            self.pipeline.stop()
            cv2.destroyAllWindows()

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    catch_system = CatchSystem(cfg)
    catch_system.run()

if __name__ == "__main__":
    main()
