from ultralytics import YOLO

path_to_model = "runs/obb/working-model/weights/best.pt"

model = YOLO(path_to_model)

results = model(source=0, show=True, imgsz=640)