#!/usr/bin/env python3
import cv2
from ultralytics import YOLO

# Load the model
model = YOLO("best.pt")

# Open an image file
img = cv2.imread('camera_.png')

# Perform predictions
pred = model.predict(img, verbose=False)

# Get the bounding boxes, confidences, and class indices
bboxes = pred[0].boxes.xywh.tolist()
confidences = pred[0].boxes.conf.tolist()
class_indices = pred[0].boxes.cls.tolist()  # Get the class indices

# Get class names from the model
labels = pred[0].names

# PLOTTING THE BOXES AND ANNOTATIONS
for box, conf, cls_idx in zip(bboxes, confidences, class_indices):
    cx, cy, w, h = [int(i) for i in box]
    x1, y1, x2, y2 = (
        int(cx - w // 2),
        int(cy - h // 2),
        int(cx + w // 2),
        int(cy + h // 2),
    )
    # Draw the rectangle
    img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # Annotate with label and confidence
    label = f"{labels[int(cls_idx)]}: {conf:.2f}"
    img = cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the image with boxes and annotations
cv2.imshow("Detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

