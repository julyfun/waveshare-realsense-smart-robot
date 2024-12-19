import numpy as np
import cv2
from dataclasses import dataclass
import requests
from loguru import logger
from typing import List, Optional
from omegaconf import DictConfig
import hydra
import time
from object_tracker2 import ObjectTracker
from typing import Protocol

from robotoy.fast import to_degrees, to_radians
from robotoy.time_utils import precise_sleep, precise_wait

def unwrap(value, msg="Unwrap failed - value is None"):
    if value is None:
        raise ValueError(msg)
    return value

class State(Protocol):
    def execute(self, ctx: "Context") -> "State":
        raise NotImplementedError

class Delay():
    def __init__(self):
        self.start_time_sys = None
        self.delay = None

    def try_start(self, time, delay):
        if self.start_time_sys is not None:
            return
        self.start_time_sys = time
        self.delay = delay

    def ok(self, time):
        return self.start_time_sys is not None\
            and self.delay is not None\
            and time - self.start_time_sys >= self.delay


@dataclass
class ObjPos:
    time: float
    xyz: np.ndarray

class Find(State):
    def __init__(self):
        ...

    def execute(self, ctx: "Context") -> "State":
        result = ctx.object_tracker.wait_for_track_one_xyz(
            ctx.cfg.catch.labels,
            ctx.cfg.catch.area_min_limit,
            want_img=True
        )

        if result is None:
            print('result in None, maybe failed to get img')
            return self

        img_time, so_obs, so_xyz, img = result
        if img is not None:
            ctx.show_image(img)

        if so_xyz is not None:
            return Position(time.time())

        return self


class Position(State):
    def __init__(self, start_time_sys):
        self.start_time_sys = start_time_sys

    def execute(self, ctx: "Context") -> "State":
        # [img]
        result = ctx.object_tracker.wait_for_track_one_xyz(
            ctx.cfg.catch.labels,
            ctx.cfg.catch.area_min_limit,
            want_img=True
        )
        if result is None:
            print('result in None, maybe failed to get img')
            return self
        img_time, so_obs, so_xyz, img = result
        if img is not None:
            ctx.show_image(img)

        # [translation]
        time_sys = time.time()
        if so_xyz is not None and time_sys - self.start_time_sys > ctx.cfg.catch.position.time:
            return GetReady(time.time(), ObjPos(img_time, so_xyz))

        return self

# class GetReady(State):
#     def __init__(self, t, obj_pos: ObjPos):
#         self.start_time_sys = t
#         self.obj_pos = obj_pos
#         self.req_sent = False

#     def execute(self, ctx: "Context") -> "State":
#         # [grip]
#         if not self.req_sent:
#             ctx.req_grip(to_radians(ctx.cfg.catch.get_ready.degree))
#             self.req_sent = True

#         # [translation]
#         if time.time() - self.start_time_sys > ctx.cfg.catch.get_ready.time:
#             return Reach(self.obj_pos)
#         return self

class Reach(State):
    def __init__(self, obj_pos: ObjPos):
        self.start_time_sys = time.time()

        self.obj_pos = obj_pos
        self.tcp_target_origin: Optional[np.ndarray] = None
        self.tcp_target_clipped: Optional[np.ndarray] = None
        self.next_req_time_sys = -1e9

    def execute(self, ctx: "Context") -> "State":
        if self.tcp_target_origin is None or self.tcp_target_clipped is None:
            # make sure it's blocked
            cfg0 = ctx.cfg.catch.reach
            ctx.pub_obj_pos(self.obj_pos.time, *self.obj_pos.xyz)
            obj = unwrap(ctx.get_tf2("base_link", "obj"))[0]

            tcp_xy = obj[:2] + cfg0.offset.xy
            tcp_xy_norm_clip = np.clip(np.linalg.norm(tcp_xy), cfg0.clip.xy.min, cfg0.clip.xy.max)
            tcp_xy_clip = tcp_xy / np.linalg.norm(tcp_xy) * tcp_xy_norm_clip
            tar_z = obj[2] + cfg0.offset.z
            tar_z_clip = np.clip(tar_z, cfg0.clip.z.min, cfg0.clip.z.max)
            # not clipped
            self.tcp_target_origin = np.array([tcp_xy[0], tcp_xy[1], tar_z])
            self.tcp_target_clipped = np.array([tcp_xy_clip[0], tcp_xy_clip[1], tar_z_clip])

        time_sys = time.time()
        if time_sys - self.start_time_sys > ctx.cfg.catch.reach.timeout:
            return Init()
        if time_sys >= self.next_req_time_sys:
            ctx.req_move_xyz(*self.tcp_target_clipped)
            interval = ctx.cfg.catch.req_time_interval
            self.next_req_time_sys = max(time_sys + 0.5 * interval, self.next_req_time_sys + interval)
        if np.linalg.norm(unwrap(ctx.get_tf2("base_link", "hand_tcp"))[0] - self.tcp_target_origin) <= ctx.cfg.catch.error.xyz:
            return Catch(time.time())
        return self

class Catch(State):
    def __init__(self, time_start_sys):
        self.time_start_sys = time_start_sys
        self.req_sent = False

    def execute(self, ctx: "Context") -> "State":
        if not self.req_sent:
            ctx.req_grip(to_radians(ctx.cfg.catch.catch.degree))
            self.req_sent = True

        if time.time() - self.time_start_sys > ctx.cfg.catch.catch.time:
            return ToBox()

        return self

class ToBox(State):
    def __init__(self):
        self.next_req_time_sys = -1e9
        self.delay = Delay()

    def execute(self, ctx: "Context") -> "State":
        time_sys = time.time()
        tar_q = to_radians(np.array(ctx.cfg.catch.to_box.q_degree))
        if time_sys >= self.next_req_time_sys:
            ctx.req_waypoint(*tar_q)
            interval = ctx.cfg.catch.req_time_interval
            self.next_req_time_sys = max(time_sys + 0.5 * interval, self.next_req_time_sys + interval)
        logger.warning(f'{ctx.get_q(), tar_q}')
        if np.all(np.abs(unwrap(ctx.get_q()) - tar_q) <= to_radians(ctx.cfg.catch.error.q_degree)):
            self.delay.try_start(time_sys, ctx.cfg.catch.to_box.delay)
        if self.delay.ok(time_sys):
            return Release(time.time())
        return self

class Release(State):
    def __init__(self, time_start_sys):
        self.time_start_sys = time_start_sys
        self.req_sent = False

    def execute(self, ctx: "Context") -> "State":
        if not self.req_sent:
            ctx.req_grip(to_radians(ctx.cfg.catch.release.degree))
            self.req_sent = True

        if time.time() - self.time_start_sys > ctx.cfg.catch.release.time:
            return Init()
        return self

class Init(State):
    def __init__(self):
        self.next_req_time_sys = -1e9
        self.req_sent = False

    def execute(self, ctx: "Context") -> "State":
        if not self.req_sent:
            ctx.req_grip(to_radians(ctx.cfg.catch.init.gripper_degree))
            self.req_sent = True

        time_sys = time.time()
        tar_q = to_radians(np.array(ctx.cfg.catch.init.q_degree))
        if time_sys >= self.next_req_time_sys:
            ctx.req_waypoint(*tar_q)
            interval = ctx.cfg.catch.req_time_interval
            self.next_req_time_sys = max(time_sys + 0.5 * interval, self.next_req_time_sys + interval)

        if np.all(np.abs(unwrap(ctx.get_q()) - tar_q) <= to_radians(ctx.cfg.catch.error.q_degree)):
            return Find()
        return self

# public for a State
class Context():
    def __init__(self, cfg):
        self.state = Init()
        self.cfg = cfg
        self.object_tracker = ObjectTracker(480, 640, 30, None, model_name='yolov8m-seg.pt')

        self.next_run_time_sys = -1e9

    def show_image(self, img: np.ndarray):
        print(f'hey2 {img.shape}')
        cv2.namedWindow('Aligned RGB and Depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Aligned RGB and Depth', img)
        cv2.waitKey(1)  # Add this line

    def req_grip(self, x):
        url = "http://localhost:4060/grip"
        data = {
            "data": x,
        }
        response = requests.post(url, json=data)

    def pub_obj_pos(self, time, x, y, z):
        url = "http://localhost:4060/pub-tf2"
        data = {
            "time": time,
            "frame_id": "rs_arm",
            "child_frame_id": "obj",
            "xyz": [x, y, z],
            "xyzw": [0.0, 0.0, 0.0, 1.0]
        }

        response = requests.post(url, json=data)

    def get_tf2(self, tar, src):
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
        logger.error(f"Error: {data['message']}")
        return None

    def get_q(self):
        url = "http://localhost:4060/get-q"
        response = requests.post(url)
        data = response.json()
        if data["status"] == "success":
            q0, q1, q2 = data["data"]
            return q0, q1, q2
        logger.error(f"Error: {data['message']}")
        return None

    def req_move_xyz(self, x, y, z):
        """
        this service gonna make base_link to hand_tcp to (x, y, z).
        """
        url = "http://localhost:4060/move-v2"
        data = {
            "x": x,
            "y": y,
            "z": z,
        }
        response = requests.post(url, json=data)

    def req_waypoint(self, q0, q1, q2):
        url = "http://localhost:4060/waypoint"
        print(f"req_waypoint: {q0}, {q1}, {q2}")
        data = {
            "q0": q0,
            "q1": q1,
            "q2": q2,
        }
        response = requests.post(url, json=data)

    def run(self):
        try:
            last = time.time()
            while True:
                cur = time.time()
                till = max(cur, last + self.cfg.catch.run_time_interval)
                precise_sleep(till - cur)
                last = till

                def print_class_info(instance):
                    logger.info(f"Class name: {instance.__class__.__name__}")
                    for attr, value in instance.__dict__.items():
                        logger.info(f"{attr}: {value}")

                print_class_info(self.state)
                self.state = self.state.execute(self)
        finally:
            self.object_tracker.stop()
            cv2.destroyAllWindows()


@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    ctx = Context(cfg)
    ctx.run()

if __name__ == "__main__":
    main()
