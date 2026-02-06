import cv2
import numpy as np
import os

# ========= 配置 =========
IMAGE_PATH = "calib_images/img_0013.png"   # 改成你要打开的那张
INTRINSICS_PATH = "camera_intrinsics.npz"
WINDOW_W, WINDOW_H = 1200, 675            # 只影响显示大小，不影响像素坐标
ALPHA = 1.0   # 1.0 保留视野(可能有黑边)；0.0 裁剪黑边（要和你算H时一致）
# =======================

img = cv2.imread(IMAGE_PATH)
if img is None:
    raise RuntimeError(f"❌ 读不到图片: {IMAGE_PATH}")

intr = np.load(INTRINSICS_PATH)
K, dist = intr["K"], intr["dist"]

h, w = img.shape[:2]
newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=ALPHA)
img_ud = cv2.undistort(img, K, dist, None, newK)

clicked = []  # 存放 (u,v) in undistorted image

def redraw(base):
    vis = base.copy()
    # 画所有点
    for i, (x, y) in enumerate(clicked):
        cv2.circle(vis, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(vis, f"{i}:({x},{y})", (x + 6, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    # 提示
    cv2.putText(vis, "Left click: add point | R: reset | S: save points | ESC: quit",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return vis

vis = redraw(img_ud)

def on_mouse(event, x, y, flags, param):
    global vis
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked.append((int(x), int(y)))
        print(f"clicked (undistorted) pixel: u={x}, v={y}")
        vis = redraw(img_ud)
        cv2.imshow("undistorted", vis)

cv2.namedWindow("undistorted", cv2.WINDOW_NORMAL)
cv2.resizeWindow("undistorted", WINDOW_W, WINDOW_H)
cv2.setMouseCallback("undistorted", on_mouse)

cv2.imshow("undistorted", vis)

while True:
    key = cv2.waitKey(0) & 0xFF

    if key == 27:  # ESC
        break

    if key in (ord('r'), ord('R')):
        clicked.clear()
        vis = redraw(img_ud)
        cv2.imshow("undistorted", vis)
        print("reset points")

    if key in (ord('s'), ord('S')):
        out_path = "clicked_points.txt"
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(f"# image: {IMAGE_PATH}\n")
            f.write(f"# undistorted size: {w}x{h}, alpha={ALPHA}\n")
            for i, (x, y) in enumerate(clicked):
                f.write(f"{i}\t{x}\t{y}\n")
        print(f"✅ saved {len(clicked)} points to {out_path}")

cv2.destroyAllWindows()
