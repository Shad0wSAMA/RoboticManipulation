#!/usr/bin/env python3
"""
Diagnose why training may have failed or produced a non-working model.
"""

import json
import torch
from pathlib import Path
from ultralytics import YOLO
import yaml

print("="*70)
print("TRAINING DIAGNOSIS TOOL")
print("="*70)

# Check 1: Weights file
WEIGHTS_PATH = Path("runs/detect/train/weights/best.pt")
print(f"\n1. Checking weights file...")
if not WEIGHTS_PATH.exists():
    print(f"   ERROR: Weights not found at {WEIGHTS_PATH}")
    print(f"   This means training either failed or hasn't run yet.")
    print(f"   Run: python train_model.py")
    exit(1)
else:
    size_mb = WEIGHTS_PATH.stat().st_size / (1024*1024)
    print(f"   ✓ Weights found: {size_mb:.1f} MB")

# Check 2: Training logs
print(f"\n2. Checking training results...")
TRAIN_DIR = Path("runs/detect/train")
if not TRAIN_DIR.exists():
    print(f"   ERROR: Training directory not found at {TRAIN_DIR}")
    exit(1)

# Look for results.csv
results_csv = TRAIN_DIR / "results.csv"
if results_csv.exists():
    print(f"   ✓ Results file found")
    with open(results_csv, 'r') as f:
        lines = f.readlines()
    print(f"   Training had {len(lines)-1} epochs (excluding header)")
    
    # Show last few lines
    print(f"\n   Last 3 epochs:")
    for line in lines[-3:]:
        print(f"   {line.strip()}")
else:
    print(f"   Warning: No results.csv found")

# Check 3: Data config
print(f"\n3. Checking data.yaml configuration...")
DATA_YAML = Path("yolo_dataset/data.yaml")
if not DATA_YAML.exists():
    print(f"   ERROR: data.yaml not found at {DATA_YAML}")
    exit(1)

with open(DATA_YAML, 'r') as f:
    data_config = yaml.safe_load(f)

print(f"   ✓ data.yaml found")
print(f"   Classes: {data_config.get('names', {})}")
num_classes = len(data_config.get('names', {}))
print(f"   Number of classes: {num_classes}")

# Check 4: Dataset images
print(f"\n4. Checking dataset...")
for split in ['train', 'val', 'test']:
    img_dir = Path("yolo_dataset/images") / split
    label_dir = Path("yolo_dataset/labels") / split
    
    if img_dir.exists():
        img_count = len(list(img_dir.glob("*.jpg"))) + len(list(img_dir.glob("*.png")))
        label_count = len(list(label_dir.glob("*.txt")))
        print(f"   {split:5s}: {img_count:3d} images, {label_count:3d} labels")
    else:
        print(f"   {split:5s}: NOT FOUND")

# Check 5: Model information
print(f"\n5. Loading and inspecting model...")
try:
    model = YOLO(str(WEIGHTS_PATH))
    print(f"   ✓ Model loaded successfully")
    print(f"   Task: {model.task}")
    print(f"   Model: {model.model.yaml}")
    
    # Check model's internal class info
    if hasattr(model, 'names'):
        print(f"   Model classes: {model.names}")
    
except Exception as e:
    print(f"   ERROR loading model: {e}")
    exit(1)

# Check 6: Detailed label analysis
print(f"\n6. Analyzing training labels...")
train_labels = Path("yolo_dataset/labels/train")
if train_labels.exists():
    label_files = list(train_labels.glob("*.txt"))
    if label_files:
        print(f"   Total label files: {len(label_files)}")
        
        # Sample a few labels
        print(f"\n   Sample labels (first 3 files):")
        for label_file in sorted(label_files)[:3]:
            with open(label_file, 'r') as f:
                lines = f.readlines()
            print(f"\n   {label_file.name}:")
            if lines:
                for line in lines[:2]:  # First 2 labels in file
                    print(f"     {line.strip()}")
                if len(lines) > 2:
                    print(f"     ... and {len(lines)-2} more labels")
            else:
                print(f"     (empty file - no labels)")
        
        # Check for empty label files
        empty_files = [f for f in label_files if f.stat().st_size == 0]
        if empty_files:
            print(f"\n   WARNING: {len(empty_files)} empty label files found")
            print(f"   This may indicate INCLUDE_EMPTY_IMAGES was True in training")
    else:
        print(f"   ERROR: No label files found in {train_labels}")
else:
    print(f"   ERROR: Labels directory not found")

# Check 7: Test inference with debug
print(f"\n7. Testing model with very low confidence threshold...")
from glob import glob

test_imgs = sorted(glob("yolo_dataset/images/train/*.jpg"))[:1]
if test_imgs:
    test_img = test_imgs[0]
    print(f"   Testing on: {Path(test_img).name}")
    
    for conf_threshold in [0.5, 0.25, 0.1, 0.05, 0.01]:
        results = model.predict(
            source=test_img,
            conf=conf_threshold,
            device=0 if torch.cuda.is_available() else "cpu",
            verbose=False
        )
        det_count = len(results[0].boxes) if results[0].boxes is not None else 0
        print(f"   Conf={conf_threshold:.2f}: {det_count} detections")

print("\n" + "="*70)
print("DIAGNOSIS SUMMARY")
print("="*70)

print("""
If no detections even at conf=0.01:
  1. Check if training actually completed (look at runs/detect/train/results.csv)
  2. Verify training loss decreased over epochs
  3. Check if labels are formatted correctly (see sample above)
  4. Ensure training ran for enough epochs (default is 100)
  
If model has few classes but your data has many:
  1. Check data.yaml - it may have missed classes
  2. Verify all class names in annotations
  3. Retrain with correct class definitions
  
If training looks complete but model doesn't work:
  1. Training may have plateaued early
  2. Try retraining with different hyperparameters
  3. Increase EPOCHS in train_model.py
  4. Try lower BATCH size if running out of memory
""")
