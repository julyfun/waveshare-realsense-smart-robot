import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from robotoy.kalman.linear_kalman import LinearKalmanAliveApi
from typing import Optional, Tuple, List
from rs_wrapper import RealSenseWrapper
from pyrust import unwrap
from loguru import logger
from collections import deque

class ObjectTracker:
    def __init__(self, height: int, width: int, fps: int,
        device_name: Optional[str], model_name: str,
        min_depth_limit: float = 0.01,
    ):
        self.rs_wrapper = RealSenseWrapper(height, width, fps, device_name)
        logger.info('Load model...')
        self.model = YOLO(model_name)

        self.kalman = LinearKalmanAliveApi(3, 2, 0.1, 10.0, 5.0)
        self.reset_kalman()

        self.min_depth_limit = min_depth_limit

    def reset_kalman(self):
        self.kalman = LinearKalmanAliveApi(3, 2, 0.1, 10.0, 5.0)

    def get_mask_center_xyz(self, mask, depth_image, depth_intrin) -> np.ndarray:
        M = cv2.moments(mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        def bfs_depth():
            # if depth_image[cY, cX] < lim, bfs from cX, cY to find the nearest depth >= lim
            depth = depth_image[cY, cX]
            if depth >= self.min_depth_limit:
                return depth
            q = deque([(cX, cY)])
            visited = set()
            while len(q):
                x, y = q.popleft()
                if depth_image[y, x] >= self.min_depth_limit:
                    return depth_image[y, x]
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < depth_image.shape[1] and 0 <= ny < depth_image.shape[0] and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        q.append((nx, ny))
            return self.min_depth_limit
        depth = bfs_depth()

        x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth)
        return np.array([x, y, z]) / 1e3

    def wait_for_track_one_xyz(self, labels: List[str], area_limit: float, want_img=False) -> \
        Optional[Tuple[float, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]:
        """
        img_time, so_obs, so_xyz, images
        - if get empty image, return None
        - else:
            - always img_time
            - If detected, there is `so_obs`
            - If kalman is alive, there is so_xyz
        """
        wait_for_data = self.rs_wrapper.wait_for_data()
        if wait_for_data is None:
            logger.info('Failed to get img')
            return None
        img_time, color_image, depth_image, depth_intrin = wait_for_data
        if depth_image is None or color_image is None:
            logger.info('Failed to get img')
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

        def fn1():
            so_obs = None
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
            so_xyz: Optional[np.ndarray] = self.kalman.get_pos(img_time)
            return so_obs, so_xyz
        so_obs, so_xyz = fn1()

        if want_img:
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

                    M = cv2.moments(mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.putText(color_image, label, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            return img_time, so_obs, so_xyz, images

        return (img_time, so_obs, so_xyz, None)

    def stop(self):
        self.rs_wrapper.stop()
