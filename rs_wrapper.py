import pyrealsense2 as rs
import numpy as np
import cv2
import time
from loguru import logger
from typing import Optional, Tuple

class RealSenseWrapper:
    def __init__(self, height, width, fps, device_name=None):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if device_name:
            self.config.enable_device(device_name)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.pc = rs.pointcloud()

    def wait_for_data(self) -> Optional[Tuple]:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return None
        img_time = aligned_depth_frame.get_timestamp() / 1e3  # ms -> s
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())  # mm -> m
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        return img_time, color_image, depth_image, depth_intrin

    def stop(self):
        self.pipeline.stop()
