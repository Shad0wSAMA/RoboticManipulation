#!/usr/bin/env python3
"""
End-to-end YOLOv8 training script from Labelme annotations.

What it does:
1) Reads Labelme .json files (rectangles or polygons) + corresponding images
2) Converts annotations to YOLO detection labels (.txt)
3) Splits into train/val/test
4) Writes a data.yaml for Ultralytics YOLOv8
5) Trains YOLOv8 (and optionally runs validation)

Assumptions (default):
- Your raw data is in:  raw/images/*.jpg (or png)  and raw/annotations/*.json
- JSON filenames match image stem: img001.json <-> img001.jpg

Install:
  pip install ultralytics opencv-python pillow pyyaml

Run:
  python train_yolov8_from_labelme.py
"""

import os
import json
import random
import shutil
from glob import glob
from pathlib import Path
from typing import Dict, List, Tuple

import yaml
from PIL import Image

# -----------------------------
# CONFIG (edit these)
# -----------------------------
RAW_IMAGES_DIR = Path("dataset_block/images")
RAW_ANN_DIR = Path("dataset_block/annotations")

OUT_DIR = Path("yolo_dataset_block")  # output dataset root
SPLIT = {"train": 0.7, "val": 0.2, "test": 0.1}  # must sum to 1.0

# YOLO training config
MODEL = "yolov8n.pt"
IMGSZ = 640
EPOCHS = 40
BATCH = 16
CONF = 0.25  # used only for optional predict/val runs

# Reproducibility
SEED = 42

# If True: include images with no labels; creates empty .txt files
INCLUDE_EMPTY_IMAGES = True

# If True: convert non-rectangle shapes to bbox (polygon->bbox). Recommended.
CONVERT_NON_RECT_TO_BBOX = True

# -----------------------------
# Helpers
# -----------------------------
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def find_image_for_json(json_path: Path, images_dir: Path) -> Path:
    """Find corresponding image by stem; prefer imagePath if present."""
    with open(json_path, "r") as f:
        data = json.load(f)

    image_path = data.get("imagePath")
    if image_path:
        candidate = images_dir / Path(image_path).name
        if candidate.exists():
            return candidate

    stem = json_path.stem
    candidates = []
    for ext in IMG_EXTS:
        c = images_dir / f"{stem}{ext}"
        if c.exists():
            candidates.append(c)

    if not candidates:
        raise FileNotFoundError(f"No image found for {json_path.name} (stem={stem}) in {images_dir}")

    # If multiple, pick first
    return candidates[0]


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


def shape_to_bbox(shape: dict) -> Tuple[float, float, float, float]:
    """
    Returns bbox corners (x1,y1,x2,y2) from a labelme shape.
    - rectangle: 2 points
    - polygon/others: bbox from all points (if CONVERT_NON_RECT_TO_BBOX)
    """
    pts = shape.get("points", [])
    st = shape.get("shape_type", "polygon")

    if st == "rectangle" and len(pts) >= 2:
        (x1, y1), (x2, y2) = pts[0], pts[1]
        x1, x2 = sorted([x1, x2])
        y1, y2 = sorted([y1, y2])
        return x1, y1, x2, y2

    if not CONVERT_NON_RECT_TO_BBOX:
        raise ValueError(f"Non-rectangle shape encountered: {st}")

    if len(pts) < 2:
        raise ValueError(f"Not enough points to form bbox for shape_type={st}")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def convert_labelme_json_to_yolo(
    json_path: Path,
    image_path: Path,
    name_to_id: Dict[str, int],
) -> List[str]:
    """Return YOLO label lines for one image."""
    w, h = Image.open(image_path).size

    with open(json_path, "r") as f:
        data = json.load(f)

    lines: List[str] = []
    for shape in data.get("shapes", []):
        label = shape.get("label")
        if label is None:
            continue
        if label not in name_to_id:
            continue

        x1, y1, x2, y2 = shape_to_bbox(shape)

        # Normalize to YOLO (cx, cy, bw, bh)
        cx = ((x1 + x2) / 2.0) / w
        cy = ((y1 + y2) / 2.0) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        cx, cy, bw, bh = map(clamp01, (cx, cy, bw, bh))
        cls_id = name_to_id[label]
        lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

    return lines


def list_class_names(json_files: List[Path]) -> List[str]:
    names = set()
    for jf in json_files:
        with open(jf, "r") as f:
            data = json.load(f)
        for shape in data.get("shapes", []):
            if "label" in shape and shape["label"] is not None:
                names.add(shape["label"])
    return sorted(names)


def make_splits(items: List[Tuple[Path, Path]], split: Dict[str, float]) -> Dict[str, List[Tuple[Path, Path]]]:
    """
    items: list of (image_path, json_path)
    """
    assert abs(sum(split.values()) - 1.0) < 1e-6, "SPLIT must sum to 1.0"

    random.shuffle(items)
    n = len(items)
    n_train = int(n * split["train"])
    n_val = int(n * split["val"])
    # remainder goes to test
    train = items[:n_train]
    val = items[n_train : n_train + n_val]
    test = items[n_train + n_val :]
    return {"train": train, "val": val, "test": test}


def write_data_yaml(out_dir: Path, class_names: List[str]) -> Path:
    data = {
        "path": str(out_dir.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": {i: n for i, n in enumerate(class_names)},
    }
    yaml_path = out_dir / "data.yaml"
    with open(yaml_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return yaml_path


def copy_and_write_labels(
    split_name: str,
    pairs: List[Tuple[Path, Path]],
    out_dir: Path,
    name_to_id: Dict[str, int],
) -> None:
    img_out = out_dir / "images" / split_name
    lab_out = out_dir / "labels" / split_name
    ensure_dir(img_out)
    ensure_dir(lab_out)

    for img_path, json_path in pairs:
        # Copy image
        dst_img = img_out / img_path.name
        shutil.copy2(img_path, dst_img)

        # Convert labels
        yolo_lines = convert_labelme_json_to_yolo(json_path, img_path, name_to_id)

        dst_txt = lab_out / f"{img_path.stem}.txt"
        if yolo_lines or INCLUDE_EMPTY_IMAGES:
            with open(dst_txt, "w") as f:
                f.write("\n".join(yolo_lines))


def main():
    random.seed(SEED)

    # Gather all JSON files
    json_files = sorted(Path(RAW_ANN_DIR).glob("*.json"))
    if not json_files:
        raise SystemExit(f"No Labelme JSON files found in {RAW_ANN_DIR}")

    # Map JSON -> image
    pairs: List[Tuple[Path, Path]] = []
    for jf in json_files:
        img = find_image_for_json(jf, RAW_IMAGES_DIR)
        pairs.append((img, jf))

    # Build class list from JSON labels
    class_names = list_class_names(json_files)
    if not class_names:
        raise SystemExit("No labels found in JSON files (shapes[].label).")

    name_to_id = {n: i for i, n in enumerate(class_names)}
    print("Classes:", class_names)

    # Create output dirs
    ensure_dir(OUT_DIR)
    for s in ("train", "val", "test"):
        ensure_dir(OUT_DIR / "images" / s)
        ensure_dir(OUT_DIR / "labels" / s)

    # Split
    splits = make_splits(pairs, SPLIT)
    for k, v in splits.items():
        print(f"{k}: {len(v)} images")

    # Convert + copy
    for split_name, split_pairs in splits.items():
        copy_and_write_labels(split_name, split_pairs, OUT_DIR, name_to_id)

    # Write data.yaml
    yaml_path = write_data_yaml(OUT_DIR, class_names)
    print("Wrote:", yaml_path)

    # Train YOLOv8
    import torch
    from ultralytics import YOLO

    # Check if CUDA is available
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device} ({'GPU' if torch.cuda.is_available() else 'CPU'})")
    
    model = YOLO(MODEL)
    print("\nStarting training...\n")
    model.train(
        data=str(yaml_path),
        imgsz=IMGSZ,
        epochs=EPOCHS,
        batch=BATCH,
        seed=SEED,
        device=device,
    )

    # Optional: validate using best weights produced
    # You can also do this later with `yolo detect val ...`
    print("\nTraining finished.")
    print("Tip: Your best model will be in runs/detect/train*/weights/best.pt")


if __name__ == "__main__":
    main()
