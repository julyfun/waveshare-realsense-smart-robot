import numpy as np
import cv2
import requests
from loguru import logger
from typing import List, Optional
from omegaconf import DictConfig
import hydra
import time
from object_tracker2 import ObjectTracker
from pyrust import unwrap
import math

class TrackPerson:
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg

        # [object_tracker]
        self.object_tracker = ObjectTracker(480, 640, 30, None, model_name='yolov8m-seg.pt')

    def run(self):
        try:
            while True:
                so_obs, so_xyz, images = unwrap(self.object_tracker.wait_for_track_one_xyz(
                    self.cfg.person.labels,
                    self.cfg.person.area_min_limit,
                    want_img=True
                ))

                if images is not None:
                    cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('Aligned RGB and Depth', images)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                try:
                    x, y, z = unwrap(so_xyz)
                    yaw = math.degrees(math.atan2(x, z))
                    pitch = -math.degrees(math.atan2(y, z))
                    print(f"yaw: {yaw}, pitch: {pitch}")

                    data = {
                        "SOF": int(0xfe54aaaa),
                        "vx": float(np.linalg.norm([x, y, z]) / 5.0),
                        "vw": float(yaw / 360.0),
                        "tick": int(1000)
                    }
                    response = requests.post("http://127.0.0.1:4050/ttf/", json=data)
                    print(response.json())


                except Exception as e:
                    logger.error(e)

        finally:
            self.object_tracker.stop()
            cv2.destroyAllWindows()

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    catch_system = TrackPerson(cfg)
    catch_system.run()

if __name__ == "__main__":
    main()
