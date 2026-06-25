import cv2
import torch
from ultralytics import YOLO

# 1. Kontrollo pajisjen (në laptop do të jetë automatikisht CPU)
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Duke ekzekutuar në: {device.upper()}")

# 2. Ngarko modelin YOLO (do të shkarkohet automatikisht në nisjen e parë)
model = YOLO("yolov8n.pt")

# 3. Hap videon që shkarkove nga YouTube
video_path = "carla_test.mp4"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print(f"GABIM: Skedari '{video_path}' nuk u gjet në këtë dosje!")
    exit()

print("Shtypni tastin 'q' për të mbyllur dritaren e testimit.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Videoja përfundoi.")
        break
        
    # Ekzekuto YOLO: classes=0 filtron vetëm njerëzit (këmbësorët)
    results = model.predict(source=frame, device=device, conf=0.25, classes=0, verbose=False)
    
    # Merr rezultatin e parë (pasi punojmë me një imazh të vetëm çdo herë)
    result = results[0]
    
    # Shfaq në terminal koordinatat në pikselë për çdo këmbësor të gjetur
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        pika_x = int((x1 + x2) / 2)
        pika_y = int(y2)  # Kjo është pika ku këmbët e personit prekin tokën
        print(f"Këmbësor i detektuar te pikselët: X={pika_x}, Y={pika_y}")

    # Vizato kutitë jeshile rreth këmbësorëve në video
    annotated_frame = result.plot()

    cv2.namedWindow("Mock Roadside Camera - Testimi i Kembesoreve", cv2.WINDOW_AUTOSIZE)
    
    # Shfaq videon live në ekran
    cv2.imshow("Mock Roadside Camera - Testimi i Këmbësorëve", annotated_frame)
    key = cv2.waitKey(100) & 0xFF
    
    # Shtyp 'q' në tastierë për ta ndërprerë videon para kohe
    if key == ord('q'):
        break

# Pastro memorien e sistemit
cap.release()
cv2.destroyAllWindows()