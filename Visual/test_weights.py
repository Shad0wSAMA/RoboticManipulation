#!/usr/bin/env python3
"""
Test script for YOLOv8 trained weights.

What it does:
1) Loads the best trained model from runs/detect/train/weights/best.pt
2) Runs inference on test images
3) Evaluates performance metrics (mAP, precision, recall)
4) Visualizes predictions on sample images
5) Saves results to output folder

Install:
  pip install ultralytics opencv-python pillow pyyaml

Run:
  python test_weights.py
"""

import os
import cv2
from pathlib import Path
from glob import glob
import torch
from ultralytics import YOLO

# Config
WEIGHTS_PATH = Path("runs/detect/train/weights/best.pt")
TEST_IMAGES_DIR = Path("yolo_dataset/images/test")
TRAIN_IMAGES_DIR = Path("yolo_dataset/images/train")  # Also test on training images
OUTPUT_DIR = Path("test_results")
CONF_THRESHOLD = 0.25  # Try lowering this if no detections
CONF_THRESHOLD_LOW = 0.05  # Very low threshold for debugging
DEVICE = 0 if torch.cuda.is_available() else "cpu"

# Ensure output directory exists
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

print(f"Using device: {DEVICE} ({'GPU' if torch.cuda.is_available() else 'CPU'})")


def debug_model():
    """Debug why model is not detecting objects."""
    
    if not WEIGHTS_PATH.exists():
        print(f"Error: Weights not found at {WEIGHTS_PATH}")
        return
    
    print("\n" + "="*60)
    print("DEBUGGING MODEL - NO DETECTION ISSUE")
    print("="*60)
    
    model = YOLO(str(WEIGHTS_PATH))
    print(f"\n1. Model loaded successfully")
    print(f"   Model: {model}")
    print(f"   Task: {model.task}")
    
    # Get a training image to test on (should have detections)
    train_images = sorted(glob(str(TRAIN_IMAGES_DIR / "*.jpg"))) + \
                   sorted(glob(str(TRAIN_IMAGES_DIR / "*.png")))
    
    if not train_images:
        print(f"\n2. Error: No training images found in {TRAIN_IMAGES_DIR}")
        print("   Testing on training set first helps verify the model works at all.")
        return
    
    print(f"\n2. Found {len(train_images)} training images")
    print("   Testing on TRAINING set first (should have high detections)...\n")
    
    # Test on first training image
    test_img = train_images[0]
    print(f"   Testing on: {Path(test_img).name}")
    
    results = model.predict(
        source=test_img,
        conf=CONF_THRESHOLD_LOW,
        device=DEVICE,
        save=False,
        verbose=False
    )
    
    result = results[0]
    detections = len(result.boxes) if result.boxes is not None else 0
    print(f"   Detections with conf={CONF_THRESHOLD_LOW}: {detections}")
    
    if detections > 0:
        print("\n3. Model IS working! It detected objects in training image.")
        print("   The issue is likely with TEST images or confidence threshold.")
        print(f"\n   Suggestions:")
        print(f"   - Lower CONF_THRESHOLD to {CONF_THRESHOLD_LOW} in config")
        print(f"   - Check if test images are similar to training images")
        print(f"   - Verify training completed successfully")
        
        # Show detection details
        print(f"\n   Detection details from training image:")
        for i, box in enumerate(result.boxes):
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            class_name = result.names[cls] if cls in result.names else f"class_{cls}"
            print(f"     {i+1}. {class_name}: conf={conf:.4f}")
    else:
        print("\n3. Model is NOT detecting even in training images!")
        print("   This suggests:")
        print("   - Training may not have completed properly")
        print("   - Model may not have converged")
        print("   - Check training logs in runs/detect/train/")
        print("   - Try retraining with more epochs or different hyperparameters")
    
    print("\n" + "="*60 + "\n")
def test_model():
    """Load model and run inference on test images."""
    
    # Check if weights exist
    if not WEIGHTS_PATH.exists():
        print(f"Error: Weights not found at {WEIGHTS_PATH}")
        print("Please run train_model.py first to generate weights.")
        return
    
    print(f"\nLoading model from {WEIGHTS_PATH}...")
    model = YOLO(str(WEIGHTS_PATH))
    
    # Get test images
    test_images = sorted(glob(str(TEST_IMAGES_DIR / "*.jpg"))) + \
                  sorted(glob(str(TEST_IMAGES_DIR / "*.png")))
    
    if not test_images:
        print(f"Error: No test images found in {TEST_IMAGES_DIR}")
        return
    
    print(f"Found {len(test_images)} test images")
    print("\nRunning inference...\n")
    
    # Run inference on all test images
    for i, img_path in enumerate(test_images, 1):
        print(f"[{i}/{len(test_images)}] Processing {Path(img_path).name}...")
        
        # Run inference
        results = model.predict(
            source=img_path,
            conf=CONF_THRESHOLD,
            device=DEVICE,
            save=False,
            verbose=False
        )
        
        # Get the result
        result = results[0]
        
        # Read original image
        img = cv2.imread(img_path)
        
        # Draw predictions on image
        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                class_name = result.names[cls]
                
                # Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label with confidence
                label = f"{class_name}: {conf:.2f}"
                cv2.putText(
                    img, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )
        else:
            cv2.putText(
                img, "No detections", (30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )
        
        # Save result
        output_path = OUTPUT_DIR / f"result_{Path(img_path).stem}.jpg"
        cv2.imwrite(str(output_path), img)
        print(f"  Saved to {output_path}")
    
    print("\nInference complete!")


def validate_model():
    """Validate model on test set and compute metrics."""
    
    if not WEIGHTS_PATH.exists():
        print(f"Error: Weights not found at {WEIGHTS_PATH}")
        return
    
    print(f"\nLoading model from {WEIGHTS_PATH}...")
    model = YOLO(str(WEIGHTS_PATH))
    
    # Validate on test set
    print("\nRunning validation...\n")
    metrics = model.val(
        data="yolo_dataset/data.yaml",
        imgsz=1280,
        batch=16,
        device=DEVICE,
        verbose=True
    )
    
    # Print metrics
    print("\n" + "="*50)
    print("VALIDATION METRICS")
    print("="*50)
    print(f"mAP50: {metrics.box.map50:.4f}")
    print(f"mAP50-95: {metrics.box.map:.4f}")
    print(f"Precision: {metrics.box.mp:.4f}")
    print(f"Recall: {metrics.box.mr:.4f}")
    print("="*50 + "\n")


def predict_single_image(image_path):
    """Predict on a single image."""
    
    if not WEIGHTS_PATH.exists():
        print(f"Error: Weights not found at {WEIGHTS_PATH}")
        return
    
    if not Path(image_path).exists():
        print(f"Error: Image not found at {image_path}")
        return
    
    print(f"\nLoading model from {WEIGHTS_PATH}...")
    model = YOLO(str(WEIGHTS_PATH))
    
    print(f"Running inference on {image_path}...\n")
    results = model.predict(
        source=image_path,
        conf=CONF_THRESHOLD,
        device=DEVICE,
        save=False,
        verbose=True
    )
    
    # Process results
    result = results[0]
    
    # Read and annotate image
    img = cv2.imread(image_path)
    
    if result.boxes is not None and len(result.boxes) > 0:
        print(f"\nDetected {len(result.boxes)} objects:")
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            class_name = result.names[cls]
            print(f"  - {class_name}: {conf:.2f}")
            
            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label with confidence
            label = f"{class_name}: {conf:.2f}"
            cv2.putText(
                img, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )
    else:
        print("No objects detected.")
        cv2.putText(
            img, "No detections", (30, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )
    
    # Save annotated image
    output_path = OUTPUT_DIR / f"result_{Path(image_path).stem}.jpg"
    cv2.imwrite(str(output_path), img)
    print(f"\nAnnotated image saved to: {output_path}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "debug":
        # Debug mode - test on training images
        debug_model()
    elif len(sys.argv) > 1 and sys.argv[1] == "validate":
        # Run validation on test set
        validate_model()
    elif len(sys.argv) > 1 and sys.argv[1] != "infer":
        # Predict on single image
        predict_single_image(sys.argv[1])
    else:
        # Run inference on all test images
        test_model()
        print("\nUsage:")
        print("  python test_weights.py              - Test on all test images")
        print("  python test_weights.py debug        - Debug: test on training images")
        print("  python test_weights.py validate     - Run validation metrics")
        print("  python test_weights.py <image_path> - Test single image")
