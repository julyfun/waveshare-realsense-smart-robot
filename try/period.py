import requests
import math
import time

# Constants
url = "http://localhost:4060/move-v2"
headers = {'Content-Type': 'application/json'}
radius = 0.3
z = 0.3
period = 5  # seconds
step = 10  # degrees
angles = list(range(-90, 91, step)) + list(range(90, -91, -step))

# Function to convert degrees to radians
def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)

# Function to calculate x and y based on radius and angle
def calculate_coordinates(radius, angle):
    radians = degrees_to_radians(angle)
    x = radius * math.cos(radians)
    y = radius * math.sin(radians)
    return x, y

# Main loop to send requests
while True:
    for angle in angles:
        x, y = calculate_coordinates(radius, angle)
        data = {
            "x": x,
            "y": y,
            "z": z
        }
        response = requests.post(url, headers=headers, json=data)
        print(f"Sent data: {data}, Response: {response.status_code}")
        time.sleep(period / len(angles))
