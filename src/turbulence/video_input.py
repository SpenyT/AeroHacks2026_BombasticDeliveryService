import cv2

class Cam():
    def __init__(self, index: int, name: str):
        self.index = index
        self.name = name
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)

        if not self.cap.isOpened():
            raise RuntimeError(f"Error: Could not open {self.name} (index {self.index})")
        
        print(f"{self.name} initialized successfully!")

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError(f"Error: Could not read frame from {self.name}")
        return frame

    def release(self):
        self.cap.release()
        print(f"{self.name} released")

    def __del__(self):
        if self.cap.isOpened():
            self.release()


# change index (number) to get correct video feeds
def video_input():
    return Cam(0, "Front Cam"), Cam(1, "Side Cam")


# TEST FUNC
def test_cam():
    return Cam(1, "Test Cam");
