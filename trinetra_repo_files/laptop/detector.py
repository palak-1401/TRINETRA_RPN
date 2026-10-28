"""Laptop-side vision: read the smartphone video stream, find the target and
send 'V,<detected>,<x_offset>,<size>' packets to the Arduino over USB serial.

Three ways to define the target:
  1. Colour (default):      --color red|orange|yellow|green|blue
  2. Click-to-calibrate:    --calibrate   (click the object in the preview window)
  3. YOLO model:            --model yolov8n.pt --classes bottle,cup
                            --model runs/detect/train/weights/best.pt   (your own trained model)

Examples:
  python laptop/detector.py --stream http://192.168.1.5:8080/video --port COM3
  python laptop/detector.py --stream 0 --no-serial --show --calibrate
  python laptop/detector.py --stream 0 --no-serial --show --model yolov8n.pt --classes bottle
Press 'q' in the preview window (or Ctrl+C) to quit.

x_offset: -100 (far left) .. 100 (far right) from the frame centre
size:     target width as a percentage of frame width (distance proxy).
          Re-tune STOP_SIZE / SLOW_SIZE in the firmware for your object's real width.
"""
import argparse
import sys
import time

import cv2
import numpy as np

try:
    import serial
except ImportError:       # allows --no-serial without pyserial
    serial = None

# HSV ranges (OpenCV: H 0-179). Tune for your target and lighting.
COLORS = {
    "red": [((0, 120, 80), (10, 255, 255)), ((170, 120, 80), (179, 255, 255))],
    "orange": [((8, 120, 100), (20, 255, 255))],
    "yellow": [((20, 100, 100), (35, 255, 255))],
    "green": [((40, 80, 60), (85, 255, 255))],
    "blue": [((95, 120, 60), (130, 255, 255))],
}


def packet_values(bbox, w):
    """Convert a bounding box (x, y, bw, bh) into (x_offset, size)."""
    x, _, bw, _ = bbox
    x_offset = int(round((x + bw / 2 - w / 2) / (w / 2) * 100))
    size = int(round(min(100, bw / w * 100)))
    return max(-100, min(100, x_offset)), size


def detect_color(frame, ranges, min_area_frac=0.002):
    """Return (detected, x_offset, size, bbox) for the largest blob in the colour ranges."""
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    mask = np.zeros((h, w), np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, 0, 0, None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area_frac * w * h:
        return 0, 0, 0, None
    bbox = cv2.boundingRect(c)
    x_off, size = packet_values(bbox, w)
    return 1, x_off, size, bbox


def ranges_from_click(frame, px, py, patch=7, dh=10, ds=70, dv=70):
    """Build HSV ranges around the colour under pixel (px, py)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    y0, y1 = max(0, py - patch), py + patch + 1
    x0, x1 = max(0, px - patch), px + patch + 1
    h, s, v = (int(np.median(hsv[y0:y1, x0:x1, i])) for i in range(3))
    s_lo, s_hi = max(0, s - ds), min(255, s + ds)
    v_lo, v_hi = max(0, v - dv), min(255, v + dv)
    lo_h, hi_h = h - dh, h + dh
    if lo_h < 0:         # hue wraps around (e.g. red)
        return [((0, s_lo, v_lo), (hi_h, s_hi, v_hi)), ((180 + lo_h, s_lo, v_lo), (179, s_hi, v_hi))]
    if hi_h > 179:
        return [((lo_h, s_lo, v_lo), (179, s_hi, v_hi)), ((0, s_lo, v_lo), (hi_h - 180, s_hi, v_hi))]
    return [((lo_h, s_lo, v_lo), (hi_h, s_hi, v_hi))]


def load_yolo(path):
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ultralytics not installed: pip install ultralytics (needed for --model)")
    return YOLO(path)


def detect_yolo(model, frame, classes, conf):
    """Return (detected, x_offset, size, bbox) for the largest matching YOLO box."""
    h, w = frame.shape[:2]
    res = model(frame, verbose=False, conf=conf)[0]
    best = None
    for box in res.boxes:
        name = res.names[int(box.cls)]
        if classes and name not in classes:
            continue
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
        area = (x2 - x1) * (y2 - y1)
        if best is None or area > best[0]:
            best = (area, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
    if best is None:
        return 0, 0, 0, None
    x_off, size = packet_values(best[1], w)
    return 1, x_off, size, best[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stream", default="0", help="phone stream URL, or webcam index (default 0)")
    ap.add_argument("--port", help="Arduino serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--color", choices=COLORS, default="red", help="target colour")
    ap.add_argument("--calibrate", action="store_true",
                    help="click the object in the preview window to set its colour (implies --show)")
    ap.add_argument("--model", help="YOLO weights (e.g. yolov8n.pt or your best.pt); overrides colour")
    ap.add_argument("--classes", default="", help="comma-separated YOLO class names to follow")
    ap.add_argument("--conf", type=float, default=0.4, help="YOLO confidence threshold")
    ap.add_argument("--max-hz", type=float, default=20.0, help="packets per second")
    ap.add_argument("--no-serial", action="store_true", help="print packets, do not use Arduino")
    ap.add_argument("--show", action="store_true", help="show preview window")
    args = ap.parse_args()
    if args.calibrate:
        args.show = True

    if not args.no_serial:
        if serial is None:
            sys.exit("pyserial not installed: pip install pyserial (or use --no-serial)")
        if not args.port:
            sys.exit("--port is required unless --no-serial is used")

    src = int(args.stream) if args.stream.isdigit() else args.stream
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"Cannot open video source: {args.stream}")

    model = load_yolo(args.model) if args.model else None
    classes = {c.strip() for c in args.classes.split(",") if c.strip()}

    ser = None
    if not args.no_serial:
        ser = serial.Serial(args.port, args.baud, timeout=0)
        time.sleep(2.0)       # Arduino resets when the port opens

    state = {"ranges": COLORS[args.color], "click": None}
    if args.show:
        cv2.namedWindow("Trinetra detector")
        if args.calibrate:
            cv2.setMouseCallback("Trinetra detector", lambda e, x, y, f, p:
                                 state.update(click=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
            print("Calibrate: click on the object in the preview window.")

    period = 1.0 / args.max_hz
    last_send = 0.0
    failures = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures >= 10:
                    print("Video stream lost. Stopping (Arduino failsafe will halt motors).")
                    break
                continue
            failures = 0

            if state["click"]:
                px, py = state["click"]
                state["click"] = None
                state["ranges"] = ranges_from_click(frame, px, py)
                print("Calibrated HSV ranges:", state["ranges"])

            if model:
                detected, x_off, size, bbox = detect_yolo(model, frame, classes, args.conf)
            else:
                detected, x_off, size, bbox = detect_color(frame, state["ranges"])

            now = time.time()
            if now - last_send >= period:
                line = f"V,{detected},{x_off},{size}"
                if ser:
                    ser.write((line + "\n").encode())
                else:
                    print(line)
                last_send = now

            if ser and ser.in_waiting:
                status = ser.read(ser.in_waiting).decode(errors="ignore").strip()
                if status:
                    print("Arduino:", status.splitlines()[-1])

            if args.show:
                if bbox:
                    x, y, bw, bh = bbox
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(frame, f"V,{detected},{x_off},{size}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                cv2.imshow("Trinetra detector", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.write(b"V,0,0,0\n")
            ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
