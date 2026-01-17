#!/usr/bin/env python3
"""
ML Pipeline Node for Crater Detection

Runs YOLOv5 on Hailo accelerator and publishes detection results to ROS2.

Publishes:
    /ml_pipeline (String) - "NO_CRATER" or "CRATER x1=... y1=... x2=... y2=... conf=..."

Usage: ros2 run rover ml_pipeline

NOTE: This node requires the Hailo Python environment to be active!
      Run with: source ~/venv_hailo310/bin/activate && ros2 run rover ml_pipeline
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
import subprocess
import time

# Check for Hailo - import only if available
try:
    from hailo_platform import (
        HEF, VDevice, InferVStreams,
        InputVStreamParams, OutputVStreamParams, FormatType
    )
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False
    print("WARNING: Hailo platform not available. Running in simulation mode.")


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


class MLPipelineNode(Node):
    def __init__(self):
        super().__init__('ml_pipeline')
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(String, '/ml_pipeline', 10)
        
        self.get_logger().info('=== ML Pipeline Node Started ===')
        
        if not HAILO_AVAILABLE:
            self.get_logger().warn('Hailo not available - running simulation mode')
            self.timer = self.create_timer(1.0, self.simulation_callback)
            return
        
        # Start camera
        self.get_logger().info('Starting rpicam-vid...')
        self.cam = subprocess.Popen([
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
        
        # Output streamer for visualization
        self.ffmpeg = subprocess.Popen(
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
        
        # Input stream
        self.cap = cv2.VideoCapture(IN_STREAM, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open MJPEG stream!')
            return
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Hailo setup
        self.hef = HEF(HEF_PATH)
        self.device = VDevice()
        self.ng = self.device.configure(self.hef)[0]
        self.ins = InputVStreamParams.make(self.ng, FormatType.UINT8)
        self.outs = OutputVStreamParams.make(self.ng, FormatType.UINT8)
        self.input_name = next(iter(self.ins.keys()))
        
        self.ng.activate()
        self.infer = InferVStreams(self.ng, self.ins, self.outs)
        self.infer.__enter__()
        
        self.get_logger().info('ML Pipeline running!')
        
        # Timer for inference loop (run at ~12 FPS)
        self.timer = self.create_timer(1.0 / FPS, self.inference_callback)
    
    def simulation_callback(self):
        """Simulation mode - alternates between detection and no detection."""
        import random
        if random.random() > 0.5:
            msg = String()
            msg.data = f"CRATER x1=100 y1=200 x2=300 y2=400 conf=0.85"
            self.detection_pub.publish(msg)
        else:
            msg = String()
            msg.data = "NO_CRATER"
            self.detection_pub.publish(msg)
    
    def inference_callback(self):
        """Run inference and publish results."""
        # Grab and discard old frames
        self.cap.grab()
        self.cap.grab()
        ret, frame = self.cap.read()
        
        if not ret:
            return
        
        # Preprocess
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
        
        # Inference
        out = self.infer.infer({self.input_name: inp})
        
        # Parse outputs
        feature_maps, objectness_maps = {}, {}
        for t in out.values():
            g = t.shape[1]
            if t.shape[-1] == 1:
                objectness_maps[g] = t[0]
            else:
                feature_maps[g] = t[0]
        
        # Decode detections
        detections = []
        for grid, reg_map in feature_maps.items():
            stride = IMG_SIZE // grid
            obj_map = objectness_maps[grid]
            
            for y in range(grid):
                for x in range(grid):
                    raw = (obj_map[y, x, 0].astype(np.float32) - 128.0) / 16.0
                    score = sigmoid(raw / TEMPERATURE)
                    if score < CONF_THRES:
                        continue
                    
                    reg = reg_map[y, x]
                    l = dfl_decode(reg[0:16])
                    t = dfl_decode(reg[16:32])
                    r = dfl_decode(reg[32:48])
                    b = dfl_decode(reg[48:64])
                    
                    cx = (x + 0.5) * stride
                    cy = (y + 0.5) * stride
                    
                    detections.append([
                        int(cx - l * stride),
                        int(cy - t * stride),
                        int(cx + r * stride),
                        int(cy + b * stride),
                        score
                    ])
        
        # NMS
        final = []
        if detections:
            boxes = [[d[0], d[1], d[2] - d[0], d[3] - d[1]] for d in detections]
            scores = [d[4] for d in detections]
            idxs = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRES, NMS_IOU)
            if len(idxs):
                for i in idxs.flatten():
                    final.append(detections[i])
        
        # Publish results
        msg = String()
        if final:
            # Take best detection
            best = max(final, key=lambda d: d[4])
            x1, y1, x2, y2, sc = best
            msg.data = f"CRATER x1={x1} y1={y1} x2={x2} y2={y2} conf={sc:.3f}"
            
            # Draw on canvas
            for x1, y1, x2, y2, sc in final:
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(canvas, f"{sc:.2f}",
                            (x1, max(15, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            msg.data = "NO_CRATER"
        
        self.detection_pub.publish(msg)
        self.get_logger().debug(msg.data)
        
        # Stream output
        try:
            self.ffmpeg.stdin.write(canvas.tobytes())
        except:
            pass
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        if hasattr(self, 'infer'):
            self.infer.__exit__(None, None, None)
        if hasattr(self, 'ng'):
            self.ng.deactivate()
        if hasattr(self, 'device'):
            self.device.__exit__(None, None, None)
        if hasattr(self, 'cam'):
            self.cam.terminate()
        if hasattr(self, 'ffmpeg'):
            self.ffmpeg.terminate()
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MLPipelineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
