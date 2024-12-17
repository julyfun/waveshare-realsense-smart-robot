import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from fast_math.kalman.linear_kalman import LinearKalmanAliveApi
from typing import Optional, Tuple, List
from rs_wrapper import RealSenseWrapper
from pyrust import unwrap

class ObjectTracker:
    def __init__(self, height: int, width: int, fps: int, device_name: Optional[str], model_name: str):
        self.rs_wrapper = RealSenseWrapper(height, width, fps, device_name)
        self.model = YOLO(model_name)
        self.kalman = LinearKalmanAliveApi(3, 2, 0.1, 10.0, 5.0)

    def get_mask_center_xyz(self, mask, depth_image, depth_intrin) -> np.ndarray:
        M = cv2.moments(mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        depth = depth_image[cY, cX]
        x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth)
        return np.array([x, y, z]) / 1e3

    def wait_for_track_one_xyz(self, labels: List[str], area_limit: float, want_img=False) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        img_time, color_image, depth_image, depth_intrin = unwrap(self.rs_wrapper.wait_for_data())
        if depth_image is None or color_image is None:
            return None

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
                if label in labels:
                    area = np.sum(mask)
                    obj_masks.append((mask, area))

        if len(obj_masks):
            largest_mask, largest_area = max(obj_masks, key=lambda x: x[1])
            if self.kalman.dead(img_time):
                so_obs = self.get_mask_center_xyz(largest_mask, depth_image, depth_intrin)
            else:
                kalman_xyz = self.kalman.get_pos(img_time)
                closest_xyz = None
                for m, a in obj_masks:
                    if a < area_limit * largest_area:
                        continue
                    xyz = self.get_mask_center_xyz(m, depth_image, depth_intrin)
                    if closest_xyz is None or np.linalg.norm(xyz - kalman_xyz) < np.linalg.norm(closest_xyz - kalman_xyz):
                        closest_xyz = xyz
                so_obs = unwrap(closest_xyz)

            self.kalman.update(img_time, so_obs)
            so_xyz = unwrap(self.kalman.get_pos(img_time))
            return so_obs, so_xyz
        return None

    def stop(self):
        self.rs_wrapper.stop()
