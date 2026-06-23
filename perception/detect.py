from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")  # downloads automatically first run

results = model(
    source="sim/test_video.mp4",
    show=True,
    classes=[0],        # 0 = person only
    conf=0.4,
    stream=True
)

for r in results:
    for box in r.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        conf = float(box.conf[0])
        print(f"Person at pixel ({cx:.0f}, {cy:.0f}), confidence {conf:.2f}")