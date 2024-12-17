import numpy as np
import cv2
import requests
from loguru import logger
from typing import List, Optional
from omegaconf import DictConfig
import hydra
import time
from object_tracker2 import ObjectTracker

def unwrap(value, msg="Unwrap failed - value is None"):
    if value is None:
        raise ValueError(msg)
    return value

class CatchSystem:
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg

        # [object_tracker]
        self.object_tracker = ObjectTracker(480, 640, 30, None, model_name='yolov8m-seg.pt')

        # [catch_test]
        self.catch_test_last_move = -10086.666

        # [cache]
        self.cache_cpst_z = None
        self.state_grip = "close"

    def run(self):
        try:
            while True:
                print('want img')
                result = self.object_tracker.wait_for_track_one_xyz(
                    self.cfg.catch.labels,
                    self.cfg.catch.area_min_limit,
                    want_img=True
                )
                if result is None:
                    continue

                so_obs, so_xyz, images = result
                print(so_obs, so_xyz)

                if images is not None:
                    cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('Aligned RGB and Depth', images)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                try:
                    raise NotImplementedError
                    print(so_obs)
                    def fn2():
                        url = "http://localhost:4060/pub-tf2"
                        data = {
                            "frame_id": "rs_arm",
                            "child_frame_id": "obj",
                            "xyz": so_obs.tolist(),
                            "xyzw": [0.0, 0.0, 0.0, 1.0]
                        }
                        print(f"pub-tf2: {so_obs}")

                        response = requests.post(url, json=data)

                    fn2()

                    if self.object_tracker.kalman.dead(time.time()):
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
                        self.cache_cpst_z = 0
                    obj_xyz_in_link2 = unwrap(tf2("base_link", "obj"))[0] - np.array([0.0, 0.0, self.cache_cpst_z])
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

                        send_xyz = np.array([so_xy[0], so_xy[1], np.clip(xyz_cp[2], 0.1, 0.35)])
                        print(f'move send xyz: {send_xyz}')

                        cur_tcp_xyz_in_link2 = unwrap(tf2("base_link", "hand_tcp"))[0] - np.array([0.0, 0.0, self.cache_cpst_z])
                        cur_tcp_xyz_in_link2 = unwrap(tf2("base_link", "hand_tcp"))[0] - np.array([0.0, 0.0, 0.0])
                        if np.linalg.norm(cur_tcp_xyz_in_link2 - xyz_cp) < 0.10:
                            # grip
                            if self.state_grip == "close":
                                grip(3.0)
                                self.state_grip = "open"
                        else:
                            if self.state_grip == "open":
                                grip(0.0)
                                self.state_grip = "close"

                        url = "http://localhost:4060/move-v2"
                        data = {
                            "x": send_xyz[0],
                            "y": send_xyz[1],
                            "z": send_xyz[2],
                        }
                        # this service gonna make base_link to hand_tcp to (x, y, z + cpst_z)
                        # response = requests.post(url, json=data)
                        # data = response.json()
                        # print(data)

                    if time.time() - self.catch_test_last_move > self.cfg.catch.move_time_interval:
                        fn()
                        self.catch_test_last_move = time.time()

                except Exception as e:
                    logger.error(e)

        finally:
            self.object_tracker.stop()
            cv2.destroyAllWindows()

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    catch_system = CatchSystem(cfg)
    catch_system.run()

if __name__ == "__main__":
    main()
