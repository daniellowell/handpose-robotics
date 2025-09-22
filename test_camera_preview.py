# test_camera_preview.py
import cv2
from camera_utils import open_camera

def main():
    cap = open_camera(preferred_index=None, width=640, height=480, fps=30)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break
        cv2.putText(frame, "Press ESC to quit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        cv2.imshow("Camera Preview", frame)
        if (cv2.waitKey(1) & 0xFF) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
