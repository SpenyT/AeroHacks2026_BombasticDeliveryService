import numpy as np
import cv2

from .video_input import video_input

def turbulence():
    print("Initiating Turbulence Dampening Algorithm...")
    cam1, cam2 = video_input()

    print("Press 'q' to quit")

    TARGET_HEIGHT = 480  # or any height you want

    while True:
        frame1 = cam1.read()
        frame2 = cam2.read()

        if frame1 is None or frame2 is None:
            print("Error: Could not read frame")
            break

        # Force both to exact same height
        w1 = int(frame1.shape[1] * TARGET_HEIGHT / frame1.shape[0])
        w2 = int(frame2.shape[1] * TARGET_HEIGHT / frame2.shape[0])
        frame1 = cv2.resize(frame1, (w1, TARGET_HEIGHT))
        frame2 = cv2.resize(frame2, (w2, TARGET_HEIGHT))

        combined = np.hstack((frame1, frame2))
        cv2.imshow("Cameras", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()
