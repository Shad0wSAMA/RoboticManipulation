import json
import os
from glob import glob
from PIL import Image

# ---- paths ----
labelme_json_dir = "dataset/annotations"   # folder with .json
images_dir = "dataset/images"              # folder with images
out_labels_dir = "yolo/labels"         # output YOLO labels
os.makedirs(out_labels_dir, exist_ok=True)

# Collect all labels (class names)
class_names = set()

json_files = sorted(glob(os.path.join(labelme_json_dir, "*.json")))
if not json_files:
    raise SystemExit(f"No JSON files found in {labelme_json_dir}")

# First pass: gather class names
for jf in json_files:
    with open(jf, "r") as f:
        data = json.load(f)
    for shp in data.get("shapes", []):
        class_names.add(shp["label"])

class_names = sorted(class_names)
name_to_id = {n: i for i, n in enumerate(class_names)}

# Save classes.txt (optional but handy)
os.makedirs("yolo", exist_ok=True)
with open("yolo/classes.txt", "w") as f:
    for n in class_names:
        f.write(n + "\n")

def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))

# Second pass: convert each file
for jf in json_files:
    with open(jf, "r") as f:
        data = json.load(f)

    # Determine image size
    img_file = data.get("imagePath")
    if img_file:
        img_path = os.path.join(images_dir, os.path.basename(img_file))
    else:
        # fallback: try same stem
        stem = os.path.splitext(os.path.basename(jf))[0]
        candidates = glob(os.path.join(images_dir, stem + ".*"))
        if not candidates:
            raise FileNotFoundError(f"Can't find image for {jf}")
        img_path = candidates[0]

    w, h = Image.open(img_path).size

    yolo_lines = []
    for shp in data.get("shapes", []):
        label = shp["label"]
        shape_type = shp.get("shape_type", "polygon")

        # We expect rectangles for detection
        if shape_type != "rectangle":
            # If you used polygons/lines, skip or handle separately
            # (You *can* convert polygons to bbox by taking min/max of points)
            pts = shp["points"]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)
        else:
            # Labelme rectangle stores 2 points: top-left and bottom-right (not guaranteed order)
            (x1, y1), (x2, y2) = shp["points"]
            x1, x2 = sorted([x1, x2])
            y1, y2 = sorted([y1, y2])

        # Convert to YOLO normalized cx, cy, bw, bh
        cx = ((x1 + x2) / 2.0) / w
        cy = ((y1 + y2) / 2.0) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        cx, cy, bw, bh = map(clamp01, (cx, cy, bw, bh))

        cls_id = name_to_id[label]
        yolo_lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

    out_txt = os.path.join(out_labels_dir, os.path.splitext(os.path.basename(jf))[0] + ".txt")
    with open(out_txt, "w") as f:
        f.write("\n".join(yolo_lines))

print("Done.")
print("Classes:", class_names)
print("YOLO labels written to:", out_labels_dir)
