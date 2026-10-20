from ultralytics import YOLO

model_to_build = "yolo26n-obb.yaml"

model = YOLO(model_to_build)

results = model.train(data="data.yaml", epochs=5000)

print(results)
