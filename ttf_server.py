from fastapi import FastAPI, Depends
from omegaconf import DictConfig
import hydra
from .data_class import Flts
from .find_serial_tty import find_device_by_serial
import struct
import serial
from loguru import logger

class MyApp:
    def __init__(self, cfg: DictConfig):
        self.app = FastAPI()
        self.setup_routes()

        self.serial_number = cfg.serial_number
        self.device_path = find_device_by_serial(self.serial_number)
        self.struct_format = '<ffff'  # Four 32-bit floats
        # ...

    def setup_routes(self):
        @self.app.post("/ttf/")
        async def ttf(float_list: Flts):
            assert(len(float_list.flts) == 4)
            packed = struct.pack(self.struct_format, *float_list.flts)
            with serial.Serial(self.device_path, baudrate=9600, timeout=1) as ser:
                ser.write(packed)
                logger.info(f'Data sent, {float_list.flts}')
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
