import pyrealsense2 as rs

# Create a context object. This object owns the handles to all connected realsense devices.
context = rs.context()

# Get a list of all the connected devices
devices = context.query_devices()

# Check if there are any connected devices
if len(devices) == 0:
    print("No RealSense devices connected.")
else:
    for i, device in enumerate(devices):
        print(f"Device {i}:")
        print(f"  Name: {device.get_info(rs.camera_info.name)}")
        print(f"  Serial Number: {device.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")

