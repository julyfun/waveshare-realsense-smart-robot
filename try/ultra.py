from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO("yolo11n.pt")

results = model("https://ultralytics.com/images/bus.jpg")
