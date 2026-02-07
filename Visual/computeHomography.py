import cv2
import numpy as np
import os

# ====== 配置 ======
CAM_ID = 2
# 先填你“认为”的内角点数（cols, rows）。脚本会自动再试 (rows, cols)
PATTERN = (9, 6)
SQUARE_MM = 10.0  # 1cm = 10mm

IMG_W = 1920
IMG_H = 1080
# ==================

intr = np.load("camera_intrinsics.npz")
K, dist = intr["K"], intr["dist"]

def make_world_points(pattern_size, square_mm):
    cols, rows = pattern_size  # OpenCV pattern_size 是 (cols, rows)
    objp = np.zeros((rows * cols, 2), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)  # x最快
    objp *= square_mm
    return objp


def try_find(gray, pattern_size):
    # 更鲁棒的 SB 检测
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
    return found, corners

cap = cv2.VideoCapture(CAM_ID)
if not cap.isOpened():
    raise RuntimeError("❌ 打不开相机，请检查 CAM_ID")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)

# 预热
for _ in range(5):
    cap.read()

cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("preview", 960, 540)

print("按 S：检测成功时保存 homography_table.npz")
print("按 ESC：退出")

saved = False
best = None  # (pattern_size, corners, frame_ud)



while True:
    ok, frame = cap.read()
    if not ok:
        continue

    h, w = frame.shape[:2]

    # 用 alpha=1.0 先不裁剪，避免棋盘靠边被裁掉
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=1.0)
    frame_ud = cv2.undistort(frame, K, dist, None, newK)

    gray = cv2.cvtColor(frame_ud, cv2.COLOR_BGR2GRAY)

    # 试两种pattern（防止横纵写反）
    found1, corners1 = try_find(gray, PATTERN)
    found2, corners2 = try_find(gray, (PATTERN[1], PATTERN[0]))

    found = False
    use_pattern = None
    corners = None

    if found1:
        found, use_pattern, corners = True, PATTERN, corners1
    elif found2:
        found, use_pattern, corners = True, (PATTERN[1], PATTERN[0]), corners2

    vis = frame_ud.copy()
    cv2.putText(vis, f"{w}x{h}  found={found}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0) if found else (0,0,255), 2)

    if found:
        cv2.drawChessboardCorners(vis, use_pattern, corners, True)
        best = (use_pattern, corners, frame_ud)
        cv2.putText(vis, f"pattern={use_pattern}  Press S to save",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
    else:
        cv2.putText(vis, f"pattern tried: {PATTERN} and {(PATTERN[1], PATTERN[0])}",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

    cv2.imshow("preview", vis)
    key = cv2.waitKey(1) & 0xFF

    if key == 27:  # ESC
        break

    if key in (ord('s'), ord('S')):
        if best is None:
            print("❌ 还没检测到角点，不能保存。请调整棋盘位置/光照/距离。")
            continue

        use_pattern, corners, _ = best

        world_pts = make_world_points(use_pattern, SQUARE_MM).astype(np.float32)
        
        # print("inner corner (0,0) pixel =", corners[0][0])
        # print("frame_ud size:", frame_ud.shape[:2])  # (h,w)
        img_pts = corners.reshape(-1, 2).astype(np.float32)

        H, _ = cv2.findHomography(world_pts, img_pts)
        H_inv = np.linalg.inv(H)

        p0 = corners[0][0]
        w0 = cv2.perspectiveTransform(
            np.array([[[p0[0], p0[1]]]], np.float32),
            H_inv
        )[0,0]
        print("corner[0] ->", w0)

        np.savez(
            "homography_table.npz",
            H=H, H_inv=H_inv,
            pattern_size=np.array(use_pattern),
            square_size=np.array(SQUARE_MM),
            image_size=np.array([w, h]),
            newK=newK
        )
        print("✅ 已保存 homography_table.npz")
        saved = True
        
        break

cap.release()
cv2.destroyAllWindows()

if not saved:
    print("未保存 homography_table.npz（你按了ESC退出）")
