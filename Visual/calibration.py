import glob
import cv2
import numpy as np

# 棋盘格“内角点”数量，例如 9x6
pattern_size = (9, 6)
square_size_mm = 10  # 每个格子的边长（mm），按你的实际填

# 准备世界坐标系下的棋盘格角点 (Z=0)
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size_mm

objpoints = []
imgpoints = []

images = glob.glob("calib_images/*.png")
assert len(images) > 10, "标定图太少了，建议 >=20 张"

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not found:
        continue

    # 亚像素精炼
    corners2 = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    objpoints.append(objp)
    imgpoints.append(corners2)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("RMS reprojection error:", ret)
print("K:\n", K)
print("dist:\n", dist.ravel())

# 保存
np.savez("camera_intrinsics.npz", K=K, dist=dist)
