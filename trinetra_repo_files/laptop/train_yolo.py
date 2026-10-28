"""Train a YOLO model on your own object photos (run on Google Colab or a PC).

Steps:
  1. Take 100-300 photos of your object (different angles, distances, lighting).
  2. Label them with Roboflow or LabelImg and export in "YOLOv8" format.
     The export contains a data.yaml file.
  3. Run:
       pip install ultralytics
       python laptop/train_yolo.py --data path/to/data.yaml --epochs 50
  4. Use the trained weights with the detector:
       python laptop/detector.py --stream 0 --no-serial --show --model runs/detect/train/weights/best.pt
"""
import argparse
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="path to data.yaml from your labelled dataset")
    ap.add_argument("--base", default="yolov8n.pt", help="starting weights (small and fast)")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=640)
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ultralytics not installed: pip install ultralytics")

    model = YOLO(args.base)
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz)
    print("Done. Weights are in runs/detect/train/weights/best.pt")


if __name__ == "__main__":
    main()
