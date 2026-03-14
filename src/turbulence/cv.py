import cv2
import os
import numpy as np

def detect_bound_box(frame):
    gray_scale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_scale, (5,5), 0)
    cage_edges = cv2.Canny(blurred, 20, 150)

    kernel = np.ones((5, 5), np.uint8)
    cage_edges = cv2.dilate(cage_edges, kernel, iterations=2)
    cage_edges = cv2.erode(cage_edges, kernel, iterations=1)

    contours, _ = cv2.findContours(cage_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = None # largest box essentially
    largest_area = 0

    APPROX_ERROR = 0.05

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:
            continue

        
        
        approx = cv2.approxPolyDP(cnt, APPROX_ERROR * cv2.arcLength(cnt, True), True)
        print(f"area={area:.0f}, points={len(approx)}")
        if 4 <= len(approx) <= 8 and area > largest_area:
            largest = approx
            largest_area = area

    return largest


def draw_overlay(frame, bound_box):
    """Draw bounding box and drone position on frame."""
    if bound_box is not None:
        cv2.drawContours(frame, [bound_box], -1, (0, 255, 0), 2)

    return frame


def test_bound_box():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(base_dir, "test_images", "test.png")
    frame = cv2.imread(img_path)

    if frame is None:
        print("Error: Could not read test.png")
        return
    
    cv2.imshow("Original", frame)
    cv2.waitKey(0)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imshow("Edges", edges)
    cv2.waitKey(0)
    
    bound_box =  detect_bound_box(frame)
    if bound_box is None:
        print("No bounding box detected")
        cv2.destroyAllWindows()
        return

    frame = draw_overlay(frame, bound_box)
    print(f"Bounding box detected: {bound_box}")
    cv2.imshow("Bounding Box Test", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()