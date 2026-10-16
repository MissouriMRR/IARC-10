from ultralytics import YOLO


model = YOLO("yolo26n-obb.pt")  # load a pretrained model

results = model.train(data="./data.yaml", epochs=1)

print(results)
