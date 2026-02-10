import cv2
import os
from datetime import datetime

# ================= CONFIG =================
SAVE_DIR = "dataset_block/images"
CAMERA_INDEX = 0       # 0 = default camera
IMAGE_EXT = ".jpg"      # .jpg or .png
# ==========================================

os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open camera")

print("Camera opened.")
print("S = save image | ESC = quit")

img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Show live view
    display = frame.copy()
    cv2.putText(
        display,
        f"Images saved: {img_count}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )

    cv2.imshow("Capture (SPACE=save, ESC=quit)", display)

    key = cv2.waitKey(1) & 0xFF

    # ESC -> quit
    if key == 27:
        break

    # SPACE -> save image
    elif key == ord('s'):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"img_{timestamp}{IMAGE_EXT}"
        path = os.path.join(SAVE_DIR, filename)

        cv2.imwrite(path, frame)
        img_count += 1
        print(f"Saved: {path}")

cap.release()
cv2.destroyAllWindows()
print("Done.")
