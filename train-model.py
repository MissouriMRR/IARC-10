from ultralytics import YOLO


model_path = f"runs/obb/train/weights/best.pt"

model = YOLO(model_path)  # load the best model

model.train(data="./data.yaml",epochs=5, resume=True)

