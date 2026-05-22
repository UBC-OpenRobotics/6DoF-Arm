#!/usr/bin/env python3
"""Standalone webcam + YOLO demo — no ROS 2 required.

Run this directly on your Mac to test object detection with your webcam.
Uses the pre-trained YOLOv8n model which already detects:
  - cup, bottle (milk), spoon, bowl, knife, fork

Usage:
    pip install ultralytics opencv-python
    python scripts/webcam_demo.py

Keys:
    q - quit
    s - save screenshot
    d - toggle detection info overlay
"""

import sys
import time

import cv2


def main():
    try:
        from ultralytics import YOLO
    except ImportError:
        print('Install ultralytics first: pip install ultralytics opencv-python')
        sys.exit(1)

    # COCO classes relevant to coffee-making
    COFFEE_CLASSES = {
        'cup', 'bottle', 'spoon', 'bowl', 'knife', 'fork',
        'cell phone',  # useful for scale reference during demos
    }

    # Load pre-trained model (downloads automatically on first run)
    model = YOLO('yolov8n.pt')
    print('Model loaded. COCO classes relevant to coffee:')
    for name_id, name in model.names.items():
        if name in COFFEE_CLASSES:
            print(f'  [{name_id}] {name}')

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('Error: Cannot open webcam')
        sys.exit(1)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'Webcam: {width}x{height}')
    print('Press q to quit, s to save screenshot')

    show_info = True
    frame_count = 0
    fps_start = time.time()
    fps_display = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run YOLO detection
        results = model.predict(frame, conf=0.4, verbose=False)

        # Draw detections
        annotated = frame.copy()
        coffee_detections = []

        for result in results:
            if result.boxes is None:
                continue
            for i in range(len(result.boxes)):
                cls_name = model.names[int(result.boxes.cls[i])]
                conf = float(result.boxes.conf[i])
                xyxy = result.boxes.xyxy[i].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy

                # Highlight coffee-relevant objects in green, others in gray
                if cls_name in COFFEE_CLASSES:
                    color = (0, 255, 0)
                    coffee_detections.append((cls_name, conf, xyxy))
                else:
                    color = (128, 128, 128)

                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f'{cls_name} {conf:.2f}'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(
                    annotated,
                    (x1, y1 - label_size[1] - 10),
                    (x1 + label_size[0], y1),
                    color,
                    -1,
                )
                cv2.putText(
                    annotated, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                )

        # FPS counter
        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()

        # Info overlay
        if show_info:
            info_lines = [
                f'FPS: {fps_display:.1f}',
                f'Coffee objects: {len(coffee_detections)}',
            ]
            for det_name, det_conf, _ in coffee_detections:
                info_lines.append(f'  {det_name}: {det_conf:.2f}')

            y_offset = 30
            for line in info_lines:
                cv2.putText(
                    annotated, line, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
                )
                y_offset += 25

        cv2.imshow('6DOF Coffee Arm - Webcam Demo', annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f'demo_screenshot_{int(time.time())}.jpg'
            cv2.imwrite(filename, annotated)
            print(f'Saved: {filename}')
        elif key == ord('d'):
            show_info = not show_info

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
