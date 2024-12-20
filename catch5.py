import numpy as np
import cv2
from dataclasses import dataclass
import requests
from loguru import logger
from typing import List, Optional, cast
from omegaconf import DictConfig
import hydra
import time
from object_tracker2 import ObjectTracker
from typing import Protocol

from robotoy.fast import to_degrees, to_radians, normalized
from robotoy.fast.plane import left_dir

from robotoy.time_utils import precise_sleep, precise_wait
from robotoy.pipe import pipe, unpipe, here, UnpipeAs

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

class TimerAct:
    def __init__(self):
        self.next_act_time = -1e9

    def try_act(self, time, interval, act):
        if time >= self.next_act_time:
            act()
            self.next_act_time = max(time + 0.5 * interval, self.next_act_time + interval)

class OnceAct:
    def __init__(self):
        self.done = False

    def try_act(self, act):
        if not self.done:
            act()
            self.done = True

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
            return Position()

        return self


class Position(State):
    def __init__(self):
        self.delay = Delay()

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
        self.delay.try_start(time.time(), ctx.cfg.catch.position.delay)
        if so_xyz is not None and self.delay.ok(time.time()):
            return Reach1(ObjPos(img_time, so_xyz))
        return self

class Reach1(State):
    """
    Calculate final pos and only turn to it, and open the gripper.
    """
    def __init__(self, obj_pos: ObjPos):
        self.obj_pos = obj_pos # don't request in constructor
        self.obj_tar = None
        self.reach_tar: Optional[np.ndarray] = None
        self.stage1_tar = None
        self.delay = Delay()

        self.timer_act = TimerAct()
        self.once_act = OnceAct()

        self.start_time = time.time()

    def execute(self, ctx: "Context") -> "State":
        cfg0 = ctx.cfg.catch.reach
        if self.reach_tar is None:
            ctx.pub_obj_pos(self.obj_pos.time, *self.obj_pos.xyz)
            obj = unwrap(ctx.get_tf2("base_link", "obj"))[0]

            tcp_xy = obj[:2] + cfg0.offset.dis
            left_vec = pipe | tcp_xy | normalized | left_dir | UnpipeAs(np.ndarray)
            tcp_xy_a_little_left = tcp_xy + cfg0.offset.left * left_vec
            tcp_xy_clip = normalized(tcp_xy_a_little_left) * np.clip(np.linalg.norm(tcp_xy_a_little_left), cfg0.clip.dis.min, cfg0.clip.dis.max)
            tar_z = obj[2] + cfg0.offset.z
            tar_z_clip = pipe | tar_z | (np.clip, cfg0.clip.z.min, cfg0.clip.z.max) | unpipe

            self.obj_tar = pipe | [*tcp_xy_a_little_left, tar_z] | np.array | UnpipeAs(np.ndarray)
            self.reach_tar = pipe | [tcp_xy_clip[0], tcp_xy_clip[1], tar_z_clip] | np.array | UnpipeAs(np.ndarray)

            self.stage1_tar = np.array([
                *(
                    normalized(self.reach_tar[:2]) \
                    * np.clip(np.linalg.norm(self.reach_tar[:2]) - cfg0.stage1.xy_facing,
                        cfg0.clip.dis.min,
                        cfg0.clip.dis.max
                    )
                ), self.reach_tar[2]
            ])
            logger.warning(f"stage1_tar: {self.stage1_tar}")
            logger.warning(f"reach_tar: {self.reach_tar}")
            logger.warning(f"obj_tar: {self.obj_tar}")

        self.once_act.try_act(lambda: ctx.req_grip(to_radians(ctx.cfg.catch.reach.grip_degree)))
        self.timer_act.try_act(time.time(), ctx.cfg.catch.req_time_interval, lambda: ctx.req_move_xyz(*self.stage1_tar))

        if np.linalg.norm(unwrap(ctx.get_tf2("base_link", "hand_tcp"))[0] - self.stage1_tar) <= ctx.cfg.catch.error.dis:
            self.delay.try_start(time.time(), cfg0.stage1.delay)
        if self.delay.ok(time.time()):
            return Reach2(self.obj_tar, self.reach_tar)

        if time.time() - self.start_time > cfg0.timeout:
            return Init()

        # check pos
        return self

class Reach2(State):
    def __init__(self, obj_tar: np.ndarray, reach_tar: np.ndarray):
        self.obj_tar = obj_tar
        self.reach_tar = reach_tar
        logger.error(f"reach_tar: {reach_tar}")
        logger.error(f"obj_tar: {obj_tar}")
        self.start_time = time.time()
        self.timer_act = TimerAct()

    def execute(self, ctx: "Context") -> "State":
        self.timer_act.try_act(time.time(), ctx.cfg.catch.req_time_interval, lambda: ctx.req_move_xyz(*self.reach_tar))
        if np.linalg.norm(unwrap(ctx.get_tf2("base_link", "hand_tcp"))[0] - self.obj_tar) <= ctx.cfg.catch.error.dis:
            return Catch()
        if time.time() - self.start_time > ctx.cfg.catch.reach.timeout:
            return Init()
        return self


class Catch(State):
    def __init__(self):
        self.delay = Delay()
        self.once_act = OnceAct()

    def execute(self, ctx: "Context") -> "State":
        self.once_act.try_act(lambda: ctx.req_grip(to_radians(ctx.cfg.catch.catch.degree)))
        self.delay.try_start(time.time(), ctx.cfg.catch.catch.delay)

        if self.delay.ok(time.time()):
            return ToBox()

        return self

class ToBox(State):
    def __init__(self):
        self.timer_act = TimerAct()
        self.delay = Delay()

    def execute(self, ctx: "Context") -> "State":
        time_sys = time.time()
        tar_q = to_radians(np.array(ctx.cfg.catch.to_box.q_degree))

        self.timer_act.try_act(
            time_sys, ctx.cfg.catch.req_time_interval,
            lambda: ctx.req_waypoint(*
                to_radians(np.array(ctx.cfg.catch.to_box.q_degree))
            )
        )

        logger.warning(f'{ctx.get_q(), tar_q}')
        if np.all(np.abs(unwrap(ctx.get_q()) - tar_q) <= to_radians(ctx.cfg.catch.error.q_degree)):
            self.delay.try_start(time_sys, ctx.cfg.catch.to_box.delay)
        if self.delay.ok(time_sys):
            return Release()
        return self

class Release(State):
    def __init__(self):
        self.once_act = OnceAct()
        self.delay = Delay()

    def execute(self, ctx: "Context") -> "State":
        self.once_act.try_act(lambda: ctx.req_grip(to_radians(ctx.cfg.catch.release.degree)))
        self.delay.try_start(time.time(), ctx.cfg.catch.release.delay)
        if self.delay.ok(time.time()):
            return Init()
        return self

class Init(State):
    def __init__(self):
        self.once_act = OnceAct()
        self.timer_act = TimerAct()

    def execute(self, ctx: "Context") -> "State":
        self.once_act.try_act(lambda: ctx.req_grip(to_radians(ctx.cfg.catch.init.gripper_degree)))
        tar_q = to_radians(np.array(ctx.cfg.catch.init.q_degree))
        self.timer_act.try_act(time.time(), ctx.cfg.catch.req_time_interval, lambda: ctx.req_waypoint(*to_radians(np.array(ctx.cfg.catch.init.q_degree))))

        if np.all(np.abs(unwrap(ctx.get_q()) - tar_q) <= to_radians(ctx.cfg.catch.error.q_degree)):
            return Find()
        return self

# public for a State
class Context():
    def __init__(self, cfg):
        self.state = Init()
        self.cfg = cfg
        self.object_tracker = ObjectTracker(480, 640, 30, None, model_name='yolov11m-seg.pt')

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
                try:
                    cur = time.time()
                    till = max(cur, last + self.cfg.catch.run_time_interval)
                    precise_sleep(till - cur)
                    last = till

                    def print_class_info(instance):
                        logger.info(f"State class: {instance.__class__.__name__}")

                    print_class_info(self.state)
                    self.state = self.state.execute(self)
                except Exception as e:
                    logger.error(f"Error: {e}")
        finally:
            self.object_tracker.stop()
            cv2.destroyAllWindows()


@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    ctx = Context(cfg)
    ctx.run()

if __name__ == "__main__":
    main()
