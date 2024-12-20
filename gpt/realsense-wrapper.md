```python
import pyrealsense2 as rs
import numpy as np
import cv2
import requests
from ultralytics import YOLO
from robotoy.kalman.linear_kalman import LinearKalmanAliveApi
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
```

Wrap the code related to realsense, with few api:

- `__init__` : pass height, width, fps & device name(str)
- wair_for_data(): return depth_image, color_image, img_time (aligned)
