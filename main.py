from ultralytics import YOLO


model = YOLO("yolo26n.pt")
results = model.train(data="./data.yaml", epochs=5)

print(results)
