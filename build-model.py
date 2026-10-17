from ultralytics import YOLO

model_to_build = "yolo26n-obb.yaml"

model = YOLO(model_to_build)  # load a pretrained model

results = model.train(data="./data.yaml", epochs=2)

print(results)
