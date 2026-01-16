#!/usr/bin/env python3
"""
main.py

End-to-end YOLO pipeline using Ultralytics only:
- Train YOLO (v5/v8 compatible API)
- Export to ONNX
- Run ONNXRuntime webcam inference

Tested with YOLOv8n / YOLOv8s.
"""

from ultralytics import YOLO
import argparse
import cv2
import numpy as np
import onnxruntime as ort
import time
from pathlib import Path

# ---------------- DEFAULTS ----------------
IMG_SIZE = 640
CONF_THRESH = 0.25
NMS_IOU = 0.45
# ------------------------------------------

def train_model(data_yaml, project, epochs, batch, device):
    model = YOLO("yolov8s.pt")   # closest modern equivalent of yolov5s

    model.train(
        data=data_yaml,
        epochs=epochs,
        imgsz=IMG_SIZE,
        batch=batch,
        device=device,
        workers=2,
        single_cls=True,
        project=project,
        name="crater_yolo_single_class",
        exist_ok=True,
        amp=True,
        plots=True
    )

def export_onnx(weights_path):
    model = YOLO(weights_path)
    model.export(format="onnx", opset=17, dynamic=False)

def run_onnx_webcam(onnx_path):
    providers = (
        ["CUDAExecutionProvider", "CPUExecutionProvider"]
        if "CUDAExecutionProvider" in ort.get_available_providers()
        else ["CPUExecutionProvider"]
    )

    session = ort.InferenceSession(str(onnx_path), providers=providers)
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Webcam not accessible")

    print("🎥 Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h0, w0 = frame.shape[:2]

        img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.expand_dims(img, axis=0).astype(np.float32) / 255.0

        start = time.time()
        preds = session.run([output_name], {input_name: img})[0]
        end = time.time()

        preds = np.squeeze(preds).T   # (8400, 5) single-class YOLOv8

        boxes = []
        scores = []

        for p in preds:
            x, y, w, h, conf = p
            if conf < CONF_THRESH:
                continue

            x1 = (x - w / 2) * w0 / IMG_SIZE
            y1 = (y - h / 2) * h0 / IMG_SIZE
            x2 = (x + w / 2) * w0 / IMG_SIZE
            y2 = (y + h / 2) * h0 / IMG_SIZE

            boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
            scores.append(float(conf))

        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESH, NMS_IOU)

        for i in indices.flatten() if len(indices) else []:
            x, y, w, h = boxes[i]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Crater", (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        fps = 1 / max(1e-6, (end - start))
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Crater Detection (Ultralytics)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ---------------- MAIN ----------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=["train", "export", "infer", "all"], required=True)
    parser.add_argument("--data_yaml", help="Path to data.yaml")
    parser.add_argument("--project", default="./runs")
    parser.add_argument("--weights", help="Path to trained .pt file")
    parser.add_argument("--onnx", help="Path to .onnx file")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=20)
    parser.add_argument("--device", default=0)

    args = parser.parse_args()

    if args.stage in ("train", "all"):
        if not args.data_yaml:
            raise ValueError("--data_yaml required for training")
        train_model(args.data_yaml, args.project, args.epochs, args.batch, args.device)

    if args.stage in ("export", "all"):
        if not args.weights:
            raise ValueError("--weights required for export")
        export_onnx(args.weights)

    if args.stage in ("infer", "all"):
        if not args.onnx:
            raise ValueError("--onnx required for inference")
        run_onnx_webcam(Path(args.onnx))

if __name__ == "__main__":
    main()
