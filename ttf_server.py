from fastapi import FastAPI, Depends
from omegaconf import DictConfig
import hydra
from .data_class import Flts, PCCommData
from .find_serial_tty import find_device_by_serial
import struct
import serial
from loguru import logger
from ctypes import Structure, c_uint32, c_float


class MyApp:
    def __init__(self, cfg: DictConfig):
        self.app = FastAPI()
        self.setup_routes()

        self.serial_number = cfg.serial_number
        self.device_path = find_device_by_serial(self.serial_number)
        self.struct_format = '=IffI'
        # ...

    def setup_routes(self):
        @self.app.post("/ttf/")
        async def ttf(data: PCCommData):
            # assert(len(float_list.flts) == 4)
            c = data.to_c_struct()
            packed = struct.pack(self.struct_format,
                c.SOF,
                c.vx,
                c.vw,
                c.tick
            )
            print(packed)
            print(c.vx, c.vw)
            with serial.Serial(self.device_path, baudrate=115200, timeout=1) as ser:
                ser.write(packed)
            return {"message": "ok"}

    def run(self, cfg: DictConfig):
        import uvicorn
        uvicorn.run(self.app, host="127.0.0.1", port=cfg.ttf_server.port)

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    app_instance = MyApp(cfg)
    app_instance.run(cfg)

if __name__ == '__main__':
    main()
