from ultralytics import YOLO

yolo = YOLO('/home/lj/project/ultralytics-main/runs/detect/train1/weights/best.pt',task ='predict')
result = yolo(source='/home/lj/图片/val/**',save=True)



