import pyudev
import glob
import serial
import struct
import time
import hydra
from omegaconf import DictConfig

from .find_serial_tty import find_device_by_serial

def send_floats_to_serial(serial_number, float_values):
    device_path = find_device_by_serial(serial_number)
    if device_path is None:
        print("Device not found.")
        return

    struct_format = '<ffff'  # Four 32-bit floats
    packed_data = struct.pack(struct_format, *float_values)

    with serial.Serial(device_path, baudrate=9600, timeout=1) as ser:
        while True:
            ser.write(packed_data)
            print("Data sent.")
            time.sleep(1)  # Wait for 1 second

@hydra.main(version_base=None, config_path=".", config_name="config")
def main(cfg: DictConfig):
    serial_number = cfg.serial_number
    float_values = [1.1, 2.2, 3.3, 4.4]  # Example floats to send
    send_floats_to_serial(serial_number, float_values)

if __name__ == '__main__':
    main()
