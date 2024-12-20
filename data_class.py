from pydantic import BaseModel
from typing import List
from ctypes import c_uint32, c_float, Structure

class Flts(BaseModel):
    flts: List[float]

class PCCommDataC(Structure):
    _fields_ = [
        ("SOF", c_uint32),
        ("vx", c_float),
        ("vw", c_float),
        ("tick", c_uint32)
    ]

class PCCommData(BaseModel):
    # uint32_t
    SOF: int
    vx: float
    vw: float
    tick: int

    def to_c_struct(self) -> PCCommDataC:
        return PCCommDataC(
            SOF=self.SOF,
            vx=self.vx,
            vw=self.vw,
            tick=self.tick
        )
