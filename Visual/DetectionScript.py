#!/usr/bin/env python3
"""
Live preview + on-demand detection snapshot.

- Streams camera feed continuously
- Press 'd' to capture ONE photo and run detection on that photo only
- Saves annotated snapshot to a separate folder
- Press 'h' to toggle real-world coords label
- Press 'q' to quit
"""

import cv2
import numpy as np
import torch
from pathlib import Path
from ultralytics import YOLO
from typing import Tuple

# Config
WEIGHTS_PATH = Path("model_weights/T13.pt")
HOMOGRAPHY_FILE = Path("homography_table.npz")
INTRINSICS_FILE = Path("camera_intrinsics.npz")
CAMERA_ID = 0
CONF_THRESHOLD = 0.25

# Output folders
PREVIEW_WINDOW_NAME = "Live Preview (press 'd' to detect, 'q' to quit)"
SNAPSHOT_DIR = Path("snapshot_detections")  # <- separate folder for labeled photos
SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)

# Device selection
DEVICE = 0 if torch.cuda.is_available() else "cpu"

print("=" * 70)
print("LIVE PREVIEW + ON-DEMAND DETECTION SNAPSHOT")
print("=" * 70)
print(f"Using device for YOLO: {'GPU (cuda:0)' if DEVICE == 0 else 'CPU'}")

# =====================================================
# Load calibration data
# =====================================================
print("\nLoading calibration data...")

try:
    cal = np.load(HOMOGRAPHY_FILE)
    H_inv = cal["H_inv"]
    Knew = cal["newK"]
    print(f"✓ Homography loaded from {HOMOGRAPHY_FILE}")
except FileNotFoundError:
    raise SystemExit(f"ERROR: {HOMOGRAPHY_FILE} not found!")

try:
    intr = np.load(INTRINSICS_FILE)
    K = intr["K"]
    dist = intr["dist"]
    print(f"✓ Camera intrinsics loaded from {INTRINSICS_FILE}")
except FileNotFoundError:
    raise SystemExit(f"ERROR: {INTRINSICS_FILE} not found!")

# =====================================================
# Coordinate transformation functions
# =====================================================

def undistort_point(u: float, v: float) -> Tuple[float, float]:
    """Convert pixel coordinates from distorted to undistorted space."""
    pts = np.array([[[u, v]]], np.float32)
    pts_ud = cv2.undistortPoints(pts, K, dist, P=Knew)
    return float(pts_ud[0, 0, 0]), float(pts_ud[0, 0, 1])


def pixel_to_table(u: float, v: float) -> Tuple[float, float]:
    """Convert pixel coordinates to table/real-world coordinates."""
    u2, v2 = undistort_point(u, v)
    p = np.array([[[u2, v2]]], np.float32)
    w = cv2.perspectiveTransform(p, H_inv)
    return float(w[0, 0, 0]), float(w[0, 0, 1])


def draw_detection(
    img: np.ndarray,
    x1: int, y1: int, x2: int, y2: int,
    class_name: str,
    conf: float,
    real_x: float,
    real_y: float,
    show_real_coords: bool = True
) -> None:
    """Draw bounding box and labels on image."""
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)

    label_pixel = f"{class_name} ({conf:.2f})"
    cv2.putText(img, label_pixel, (x1, max(20, y1 - 25)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if show_real_coords:
        label_real = f"Table: ({real_x:.1f}, {real_y:.1f})"
        cv2.putText(img, label_real, (x1, max(20, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


def run_detection_on_frame(frame_bgr: np.ndarray, show_real_coords: bool) -> tuple[np.ndarray, int]:
    """
    Run YOLO + homography on a copy of the frame and return:
    - annotated_frame
    - detection_count
    """
    annotated = frame_bgr.copy()

    results = model.predict(
        source=annotated,
        conf=CONF_THRESHOLD,
        device=DEVICE,
        verbose=False
    )

    result = results[0]
    detection_count = 0

    if result.boxes is not None and len(result.boxes) > 0:
        for box in result.boxes:
            detection_count += 1

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0].item())
            cls = int(box.cls[0].item())
            class_name = result.names[cls]

            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            real_x, real_y = pixel_to_table(cx, cy)

            draw_detection(
                annotated, x1, y1, x2, y2,
                class_name, conf,
                real_x, real_y,
                show_real_coords=show_real_coords
            )

    return annotated, detection_count


# =====================================================
# Load YOLO model
# =====================================================
print("\nLoading YOLO model...")
if not WEIGHTS_PATH.exists():
    raise SystemExit(f"ERROR: Weights not found at {WEIGHTS_PATH}")

model = YOLO(str(WEIGHTS_PATH))
print(f"✓ Model loaded: {WEIGHTS_PATH}")

# =====================================================
# Open camera
# =====================================================
print(f"\nOpening camera {CAMERA_ID}...")
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise SystemExit(f"ERROR: Cannot open camera {CAMERA_ID}")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"✓ Camera opened at {w}x{h}")

print("\nControls:")
print("  d - detect on a snapshot (runs YOLO once, saves labeled image)")
print("  h - toggle table coords label")
print("  q - quit\n")

show_real_coords = True
snapshot_count = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Failed to read frame")
            break

        # Live preview text overlay
        overlay = frame.copy()
        info = f"Preview | Device: {'GPU' if DEVICE == 0 else 'CPU'} | Coords: {'ON' if show_real_coords else 'OFF'}"
        cv2.putText(overlay, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(overlay, "Press 'd' to detect snapshot, 'h' toggle coords, 'q' quit",
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)

        cv2.imshow(PREVIEW_WINDOW_NAME, overlay)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("Quitting...")
            break

        elif key == ord('h'):
            show_real_coords = not show_real_coords
            print(f"Real-world coords display: {'ON' if show_real_coords else 'OFF'}")

        elif key == ord('d'):
            # Take ONE photo (the current frame), run detection once, save annotated output
            snapshot_count += 1
            print(f"Running detection on snapshot #{snapshot_count}...")

            annotated, dets = run_detection_on_frame(frame, show_real_coords=show_real_coords)

            out_path = SNAPSHOT_DIR / f"snapshot_{snapshot_count:04d}_dets_{dets}.jpg"
            cv2.imwrite(str(out_path), annotated)

            print(f"Saved annotated snapshot: {out_path} (detections: {dets})")

            # Optional: show the annotated snapshot in a separate window
            cv2.imshow("Snapshot Detection Result", annotated)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. Labeled snapshots saved in: {SNAPSHOT_DIR.resolve()}")
