import cv2
import os

# ================= 配置 =================
SAVE_DIR = "calib_images"
CAM_ID = 0
IMG_W = 1920
IMG_H = 1080
# =======================================

def get_next_filename(folder, ext="png"):
    os.makedirs(folder, exist_ok=True)
    n = 1
    while True:
        fn = os.path.join(folder, f"img_{n:04d}.{ext}")
        if not os.path.exists(fn):
            return fn
        n += 1

def main():
    cap = cv2.VideoCapture(CAM_ID)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)

    if not cap.isOpened():
        raise RuntimeError("❌ 打不开相机，请检查 CAM_ID")

    # 预热：丢弃前几帧
    for _ in range(5):
        cap.read()

    print("📸 相机已就绪")
    print("👉 按 S 保存一张（不退出）")
    print("👉 按 ESC 退出")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        clean = frame.copy()  # 保存用的干净帧

        # 只在预览里画提示文字（不会保存）
        cv2.putText(
            frame,
            "Press S to save | ESC to quit",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )
        cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("preview", 960, 540)  # 你想要的显示大小
        cv2.imshow("preview", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            break

        if key in (ord('s'), ord('S')):
            fn = get_next_filename(SAVE_DIR, "png")
            cv2.imwrite(fn, clean)
            print(f"✅ 已保存 {fn}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
