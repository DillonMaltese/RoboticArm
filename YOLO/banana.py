from ultralytics import YOLO  # Import the YOLO class from the ultralytics library

# Load a YOLOv8 model architecture (you can choose yolov8n, yolov8s, yolov8m, yolov8l, or yolov8x)
model = YOLO("yolov8n.yaml")  # "yolov8n" means "YOLOv8 nano" – it's lightweight and fast

# Train the model using your dataset
model.train(
    data="C:\\Users\\dillo\\Documents\\git\\RoboticArm\\YOLO\\data.yaml",  # Path to your data.yaml file
    epochs=50,                 # Number of training epochs (adjust based on performance)
    imgsz=640,                 # Image size to resize to during training (default: 640)
    batch=16,                  # Number of images per training batch (adjust based on your GPU)
    name="fruit_detector"      # Name of the experiment (saves logs & weights in runs/detect/fruit_detector/)
)
