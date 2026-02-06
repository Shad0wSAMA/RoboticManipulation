import cv2
import numpy as np

cal = np.load("homography_table.npz")
H_inv = cal["H_inv"]

import cv2, numpy as np

cal = np.load("homography_table.npz")
H_inv = cal["H_inv"]
Knew = cal["newK"]

# 注意：这里需要你原始的 K 和 dist（跟标定用的同一份）
intr = np.load("camera_intrinsics.npz")
K, dist = intr["K"], intr["dist"]

def undistort_point(u, v):
    pts = np.array([[[u, v]]], np.float32)
    # 把原始像素点去畸变到 newK 坐标系下
    pts_ud = cv2.undistortPoints(pts, K, dist, P=Knew)
    return pts_ud[0,0]

def pixel_to_table(u, v):
    u2, v2 = undistort_point(u, v)
    p = np.array([[[u2, v2]]], np.float32)
    w = cv2.perspectiveTransform(p, H_inv)
    return w[0,0]

# 随便测试几个像素
tests = [(559.0546,619.59955)]
for (u,v) in tests:
    print(u, v, "->", pixel_to_table(u,v))
