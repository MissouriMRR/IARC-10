from ultralytics import YOLO


model = YOLO("yolo26n.pt")
results = model.train(epochs=5)

print(results)
