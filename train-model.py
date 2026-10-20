from ultralytics import YOLO


model_path = f"runs/obb/working-model/weights/last.pt"

model = YOLO(model_path)  # load the chosen model

model.train(data="data.yaml", epochs=100, resume=True)

