from ultralytics import YOLO


model_path = f"runs/obb/train-2/weights/last.pt"

model = YOLO(model_path)  # load the best model

model.train(data="data.yaml", epochs=100, resume=True)

