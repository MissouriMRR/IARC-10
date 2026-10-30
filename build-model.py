from ultralytics import YOLO

model_to_build = "yolo11n.yaml"

model = YOLO(model_to_build)

results = model.train(data="data.yaml", epochs=10)

print(results)
