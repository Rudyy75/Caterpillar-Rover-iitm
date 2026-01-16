import cv2
import numpy as np
import subprocess
import time
from hailo_platform import (
    HEF, VDevice, InferVStreams,
    InputVStreamParams, OutputVStreamParams, FormatType
)

# ================= CONFIG =================
HEF_PATH = "/home/rpi/models/yolov5s.hef"

CAM_PORT = "8888"
OUT_PORT = "9999"
IN_STREAM = f"tcp://127.0.0.1:{CAM_PORT}"

IMG_SIZE = 640
CONF_THRES = 0.25
NMS_IOU = 0.45
DFL_BINS = 16
TEMPERATURE = 2.0
FPS = 12
# =========================================

# ---------- utils ----------
def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

def dfl_decode(logits_u8):
    x = (logits_u8.astype(np.float32) - 128.0) / 16.0
    x -= np.max(x)
    e = np.exp(x)
    s = np.sum(e)
    if s <= 0 or not np.isfinite(s):
        return 0.0
    return float(np.sum((e / s) * np.arange(DFL_BINS)))

# ---------- start camera ----------
print("?? Starting rpicam-vid...")
cam = subprocess.Popen([
    "rpicam-vid",
    "--width", "640",
    "--height", "480",
    "--framerate", str(FPS),
    "--codec", "mjpeg",
    "--inline",
    "--timeout", "0",
    "--listen",
    "--nopreview",
    "-o", f"tcp://0.0.0.0:{CAM_PORT}"
])

time.sleep(2)

# ---------- output streamer ----------
ffmpeg = subprocess.Popen(
    [
        "ffmpeg",
        "-loglevel", "quiet",
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{IMG_SIZE}x{IMG_SIZE}",
        "-r", str(FPS),
        "-i", "-",
        "-f", "mjpeg",
        f"tcp://0.0.0.0:{OUT_PORT}?listen=1"
    ],
    stdin=subprocess.PIPE
)

# ---------- input stream ----------
cap = cv2.VideoCapture(IN_STREAM, cv2.CAP_FFMPEG)
assert cap.isOpened(), "? Cannot open MJPEG stream"
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# ---------- Hailo ----------
hef = HEF(HEF_PATH)
with VDevice() as v:
    ng = v.configure(hef)[0]
    ins = InputVStreamParams.make(ng, FormatType.UINT8)
    outs = OutputVStreamParams.make(ng, FormatType.UINT8)
    input_name = next(iter(ins.keys()))

    with ng.activate():
        with InferVStreams(ng, ins, outs) as infer:
            print("?? SINGLE-PIPELINE RUNNING")

            while True:
                cap.grab()
                cap.grab()
                ret, frame = cap.read()
                if not ret:
                    continue

                h, w = frame.shape[:2]
                scale = IMG_SIZE / max(h, w)
                nh, nw = int(h * scale), int(w * scale)
                resized = cv2.resize(frame, (nw, nh))

                canvas = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
                y0 = (IMG_SIZE - nh) // 2
                x0 = (IMG_SIZE - nw) // 2
                canvas[y0:y0+nh, x0:x0+nw] = resized

                rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
                inp = np.expand_dims(rgb, axis=0).astype(np.uint8)

                out = infer.infer({input_name: inp})

                feature_maps, objectness_maps = {}, {}
                for t in out.values():
                    g = t.shape[1]
                    if t.shape[-1] == 1:
                        objectness_maps[g] = t[0]
                    else:
                        feature_maps[g] = t[0]

                detections = []
                for grid, reg_map in feature_maps.items():
                    stride = IMG_SIZE // grid
                    obj_map = objectness_maps[grid]

                    for y in range(grid):
                        for x in range(grid):
                            raw = (obj_map[y,x,0].astype(np.float32)-128.0)/16.0
                            score = sigmoid(raw / TEMPERATURE)
                            if score < CONF_THRES:
                                continue

                            reg = reg_map[y,x]
                            l = dfl_decode(reg[0:16])
                            t = dfl_decode(reg[16:32])
                            r = dfl_decode(reg[32:48])
                            b = dfl_decode(reg[48:64])

                            cx = (x + 0.5) * stride
                            cy = (y + 0.5) * stride

                            detections.append([
                                int(cx - l*stride),
                                int(cy - t*stride),
                                int(cx + r*stride),
                                int(cy + b*stride),
                                score
                            ])

                final = []
                if detections:
                    boxes = [[d[0], d[1], d[2]-d[0], d[3]-d[1]] for d in detections]
                    scores = [d[4] for d in detections]
                    idxs = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRES, NMS_IOU)
                    if len(idxs):
                        for i in idxs.flatten():
                            final.append(detections[i])

                if final:
                    for x1,y1,x2,y2,sc in final:
                        print(f"CRATER x1={x1} y1={y1} x2={x2} y2={y2} conf={sc:.3f}")
                        cv2.rectangle(canvas,(x1,y1),(x2,y2),(0,0,255),2)
                        cv2.putText(canvas,f"{sc:.2f}",
                                    (x1,max(15,y1-5)),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                else:
                    print("NO_CRATER")

                ffmpeg.stdin.write(canvas.tobytes())