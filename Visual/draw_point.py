import cv2
import numpy as np

# ========= 你需要改的配置 =========
IMAGE_PATH = "calib_images/img_0032.png"
INTRINSICS_PATH = "camera_intrinsics.npz"
HOMO_PATH = "homography_table.npz"

USE_UNDISTORTED = True   # 如果你的 H 是在去畸变图上算的 -> True
ALPHA = 1.0              # 必须和你算 H 时一致（常用 1.0 或 0.0）

# 桌面坐标系要显示的范围（单位=你H的单位，通常是mm）
X_MIN, X_MAX = -50.0, 150.0
Y_MIN, Y_MAX = -50.0, 150.0

# 网格间距（mm）
GRID_STEP = 10.0

# 坐标轴长度（mm）
AXIS_LEN = 120.0

# 显示窗口大小（不影响像素）
WINDOW_W, WINDOW_H = 1200, 675
# =================================


def undistort_image(img, K, dist, alpha=1.0):
    h, w = img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=alpha)
    img_ud = cv2.undistort(img, K, dist, None, newK)
    return img_ud


def project_points_table_to_image(pts_xy, H):
    """
    pts_xy: (N,2) table coords
    return: (N,2) pixel coords
    """
    pts = np.asarray(pts_xy, np.float32).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, H).reshape(-1, 2)
    return out


def draw_polyline(img, pts_uv, color, thickness=2):
    pts = np.round(pts_uv).astype(int).reshape(-1, 1, 2)
    cv2.polylines(img, [pts], False, color, thickness, lineType=cv2.LINE_AA)


def draw_line_table(img, H, p0, p1, color, thickness=2):
    uv = project_points_table_to_image([p0, p1], H)
    draw_polyline(img, uv, color, thickness)


def main():
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        raise RuntimeError(f"❌ 读不到图片: {IMAGE_PATH}")

    hom = np.load(HOMO_PATH)
    if "H" not in hom:
        raise RuntimeError("❌ homography_table.npz 里没有 H（table->image）")
    H = hom["H"]

    if USE_UNDISTORTED:
        intr = np.load(INTRINSICS_PATH)
        K, dist = intr["K"], intr["dist"]
        img_show = undistort_image(img, K, dist, alpha=ALPHA)
    else:
        img_show = img.copy()

    overlay = img_show.copy()

    # -------- 1) 画网格（灰色）--------
    # 竖线：x = const
    x_vals = np.arange(X_MIN, X_MAX + 1e-6, GRID_STEP)
    y0, y1 = Y_MIN, Y_MAX
    for x in x_vals:
        draw_line_table(overlay, H, (x, y0), (x, y1), color=(180, 180, 180), thickness=1)

    # 横线：y = const
    y_vals = np.arange(Y_MIN, Y_MAX + 1e-6, GRID_STEP)
    x0, x1 = X_MIN, X_MAX
    for y in y_vals:
        draw_line_table(overlay, H, (x0, y), (x1, y), color=(180, 180, 180), thickness=1)

    # -------- 2) 画坐标轴（红色）--------
    # 原点 O=(0,0)
    O = (0.0, 0.0)
    X_end = (AXIS_LEN, 0.0)
    Y_end = (0.0, AXIS_LEN)

    draw_line_table(overlay, H, O, X_end, color=(0, 0, 255), thickness=3)  # X轴
    draw_line_table(overlay, H, O, Y_end, color=(0, 0, 255), thickness=3)  # Y轴

    # 原点画个红点
    uv_O = project_points_table_to_image([O], H)[0]
    u0, v0 = int(round(uv_O[0])), int(round(uv_O[1]))
    cv2.circle(overlay, (u0, v0), 6, (0, 0, 255), -1, lineType=cv2.LINE_AA)
    cv2.putText(overlay, "O(0,0)", (u0 + 8, v0 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

    # 标注 X/Y 轴方向
    uv_X = project_points_table_to_image([X_end], H)[0]
    ux, vx = int(round(uv_X[0])), int(round(uv_X[1]))
    cv2.putText(overlay, "+X", (ux + 8, vx - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

    uv_Y = project_points_table_to_image([Y_end], H)[0]
    uy, vy = int(round(uv_Y[0])), int(round(uv_Y[1]))
    cv2.putText(overlay, "+Y", (uy + 8, vy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

    # -------- 3) 显示 --------
    cv2.namedWindow("table_axes_grid", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("table_axes_grid", WINDOW_W, WINDOW_H)
    cv2.imshow("table_axes_grid", overlay)
    print("✅ 已叠加桌面坐标系与网格。按任意键退出。")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
