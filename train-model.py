from ultralytics import YOLO


model_path = f"runs/detect/train/weights/last.pt"

model = YOLO(model_path)  # load the chosen model

model.train(data="data.yaml", epochs=100, resume=True)

