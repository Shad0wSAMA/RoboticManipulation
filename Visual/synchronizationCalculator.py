import cv2
from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")

    cap = cv2.VideoCapture(2)  # 0=摄像头；或 "video.mp4"
    if not cap.isOpened():
        raise RuntimeError("无法打开视频源")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # stream=False 也能用；这里用单帧推理，返回 Results 列表
        results = model.predict(frame, conf=0.25, verbose=False)

        # 把预测框画到图上（Ultralytics 自带 plot）
        annotated = results[0].plot()

        cv2.imshow("YOLOv8", annotated)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
