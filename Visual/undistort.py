import cv2
import numpy as np

intr = np.load("camera_intrinsics.npz")
K, dist = intr["K"], intr["dist"]

# 假设你实际运行分辨率是 1920x1080（务必和标定时一致！）
w, h = 1920, 1080

# 生成 newK（alpha=0 裁掉黑边；alpha=1 保留视野但有黑边）
newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0.0)

map1, map2 = cv2.initUndistortRectifyMap(
    K, dist, None, newK, (w, h), m1type=cv2.CV_16SC2
)

def undistort(frame):
    return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
