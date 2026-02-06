import cv2
import numpy as np

# ========== 配置 ==========
IMAGE_PATH = "calib_images/img_0031.png"
INTRINSICS_PATH = "camera_intrinsics.npz"
HOMO_PATH = "homography_table.npz"

USE_UNDISTORTED = True
ALPHA = 1.0   # 必须和你算 H 时一致

# ====== 你要的鸟瞰图桌面范围（单位=你定义的桌面单位，通常 mm） ======
# 例如：画一个 200mm x 150mm 的桌面区域
TABLE_X_MIN = -700
TABLE_Y_MIN = -300
TABLE_X_MAX = 500
TABLE_Y_MAX = 400

# ====== 鸟瞰图分辨率（每毫米多少像素） ======
PIXELS_PER_UNIT = 3   # 3 px / mm（可调，越大越清晰）
DISPLAY_SCALE = 0.2

# ==========================


def undistort(img, K, dist, alpha=1.0):
    h, w = img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=alpha)
    return cv2.undistort(img, K, dist, None, newK)


# 读取图片
img = cv2.imread(IMAGE_PATH)
if img is None:
    raise RuntimeError("❌ 读不到图片")

# 去畸变
if USE_UNDISTORTED:
    intr = np.load(INTRINSICS_PATH)
    K, dist = intr["K"], intr["dist"]
    img = undistort(img, K, dist, ALPHA)

# 读取 Homography
hom = np.load(HOMO_PATH)
H = hom["H"]          # table -> image
H_inv = np.linalg.inv(H)

# ========= 定义鸟瞰图尺寸 =========
table_w = TABLE_X_MAX - TABLE_X_MIN
table_h = TABLE_Y_MAX - TABLE_Y_MIN

out_w = int(table_w * PIXELS_PER_UNIT)
out_h = int(table_h * PIXELS_PER_UNIT)

# ========= 构造“桌面坐标 → 鸟瞰图像素”的映射 =========
# 鸟瞰图中：
# (0,0) 像素 = (TABLE_X_MIN, TABLE_Y_MIN)
S = np.array([
    [PIXELS_PER_UNIT, 0, -TABLE_X_MIN * PIXELS_PER_UNIT],
    [0, PIXELS_PER_UNIT, -TABLE_Y_MIN * PIXELS_PER_UNIT],
    [0, 0, 1]
], dtype=np.float32)

# 最终变换：image → table → birds-eye
H_image_to_birds = S @ H_inv

# ========= 生成鸟瞰图 =========
birds = cv2.warpPerspective(
    img,
    H_image_to_birds,
    (out_w, out_h),
    flags=cv2.INTER_LINEAR
)

# ========= 可视化 =========
show_w = int(out_w * DISPLAY_SCALE)
show_h = int(out_h * DISPLAY_SCALE)

cv2.namedWindow("birds_eye", cv2.WINDOW_NORMAL)
cv2.resizeWindow("birds_eye", show_w, show_h)
cv2.imshow("birds_eye", birds)
print("✅ 鸟瞰图已生成（正视桌面）")
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存
cv2.imwrite("birds_eye_view.png", birds)
print("✅ 已保存 birds_eye_view.png")
