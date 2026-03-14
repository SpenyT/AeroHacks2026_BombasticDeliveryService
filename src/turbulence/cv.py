import cv2
import os
import numpy as np

# NOT WORKING 100% BUT WILL BE FIXING IT - Spencer
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


# ehhhhh... please test Tim, might have to alter hsv
# FYI, I chose green and blue since they are opposite on hsv cone
def detect_drone(frame, color: str = "green"):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

    if color == "green":
        mask = cv2.inRange(hsv, np.array([40, 120, 70]), np.array([80, 255, 255]))
    elif color == "blue":
        mask = cv2.inRange(hsv, np.array([100, 120, 70]), np.array([130, 255, 255]))
    else:
        print(f"Unknown color: {color}, use 'green' or 'blue'")
        return None

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    v_channel = hsv[:, :, 2]
    brightest = max(contours, key=lambda cnt: cv2.mean(v_channel, mask=cv2.drawContours(
        np.zeros_like(v_channel), [cnt], -1, 255, -1))[0])

    M = cv2.moments(brightest)
    if M["m00"] == 0:
        return None

    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def get_bbox_center(bbox):
    if bbox is None:
        return None
    x, y, w, h = cv2.boundingRect(bbox)
    return (x + w // 2, y + h // 2)




# DRAW FUNCS
def draw_bb_overlay(frame, bound_box):
    if bound_box is not None:
        cv2.drawContours(frame, [bound_box], -1, (0, 255, 0), 2)

    return frame

def draw_drone_overlay(frame, drone_pos):
    if drone_pos is None:
        print("No drone position to draw.")
        return frame
    
    cv2.circle(frame, drone_pos, 10, (0, 0, 255), -1)
    cv2.putText(frame, f"Drone: {drone_pos}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame

def draw_center_overlay(frame, center_pos):
    if center_pos is None:
        print("No center position to draw.")
        return frame
    
    cv2.circle(frame, center_pos, 6, (255, 255, 0), -1)
    return frame


def draw_overlay(frame, bound_box, drone_pos=None, center_pos=None):
    if bound_box is not None:
        cv2.drawContours(frame, [bound_box], -1, (0, 255, 0), 2)
    
    frame = draw_drone_overlay(frame, drone_pos)
    frame = draw_center_overlay(frame, center_pos)
    return frame



# TEST FUNCS
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