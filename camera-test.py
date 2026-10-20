from ultralytics import YOLO

path_to_model = "runs/obb/train/weights/best.pt"

model = YOLO(path_to_model)

results = model(source=0, show=True, imgsz=640)