from ultralytics import YOLO


model_path = f"runs/obb/train/weights/best.pt"

model = YOLO(model_path)  # load the best model

results = model.val(data="data.yaml")

print(results)