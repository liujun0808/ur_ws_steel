from ultralytics import YOLO

# load model
model = YOLO('./yolov8n.pt')
model.train(data='./workpieces.yaml', workers=1, epochs=50, batch=2)


