import cv2
from .gemini import request_bound_box_async
import numpy as np
import os

DISPLAY_HEIGHT = 480

def resize(f):
    w = int(f.shape[1] * DISPLAY_HEIGHT / f.shape[0])
    return cv2.resize(f, (w, DISPLAY_HEIGHT))

# NOT WORKING 100% BUT WILL BE FIXING IT - Spencer
# NVM, GEMINI GOT IT (LESSS GOOOOOO!)
async def async_detect_bound_box(frame):
    try:
        return await request_bound_box_async(frame)
    except Exception as e:
        print(f"[detect_bound_box] failed: {e}")
        return None

def get_bbox_center(bbox):
    x, y, w, h = bbox
    return (x + w // 2, y + h // 2)


# ehhhhh... please test Tim, might have to alter hsv
def detect_led(frame, color_str):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color_str == "red":
        lower1 = np.array([0, 120, 200])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 120, 200])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    elif color_str == "green":
        lower = np.array([35, 120, 200])
        upper = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_str == "blue":
        lower = np.array([100, 120, 200])
        upper = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_str == "white":
        lower_white = np.array([0, 0, 240])
        upper_white = np.array([180, 60, 255])
        lower_cyan = np.array([80, 30, 220])
        upper_cyan = np.array([100, 150, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white) | cv2.inRange(hsv, lower_cyan, upper_cyan)

    else:
        return None

    # removes smaller blobs of color - fyi DONT TOUCH *smacks hand*
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.dilate(mask, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    best_contour = None
    best_score = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < 2 or area > 500: # 500 not necessary prob, but wanted to remove laptop in background when I was testing so... (maybe remove?)
            continue
        c_mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(c_mask, [c], -1, 255, -1)
        mean_val = cv2.mean(gray, mask=c_mask)[0]
        score = area * mean_val
        if score > best_score:
            best_score = score
            best_contour = c

    if best_contour is None:
        return None

    M = cv2.moments(best_contour)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy)


def detect_drone(frame, colors):
    pos1 = detect_led(frame, colors[0])
    pos2 = detect_led(frame, colors[1])

    if pos1 is None or pos2 is None:
        return None

    cx = (pos1[0] + pos2[0]) // 2
    cy = (pos1[1] + pos2[1]) // 2

    return (pos1, pos2, (cx, cy))


# easier to use a dict honestly
COLOR_BGR = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "white": (255, 255, 200), # not actually white LOL
}


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
        x, y, w, h = bound_box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    frame = draw_drone_overlay(frame, drone_pos)
    frame = draw_center_overlay(frame, center_pos)

    if drone_pos is not None and center_pos is not None:
        cv2.line(frame, center_pos, drone_pos, (0, 0, 255), 2)
        offset_x = drone_pos[0] - center_pos[0]
        offset_y = drone_pos[1] - center_pos[1]
        center_x = (drone_pos[0] + center_pos[0]) // 2
        center_y = (drone_pos[1] + center_pos[1]) // 2
        cv2.putText(frame, f"dx:{offset_x} dy:{offset_y}", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    return frame

def draw_led_overlay(frame, led_pos, color):
    if led_pos is None:
        cv2.putText(frame, f"{color}: not found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return frame

    x, y = led_pos
    bgr = COLOR_BGR.get(color, (255, 255, 255))
    radius = 10
    thickness = 2

    cv2.circle(frame, (x, y), radius, bgr, thickness)
    return frame

# TEST FUNCS
def test_bound_box():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(base_dir, "test_images", "bb_test_1.jpg")
    frame = cv2.imread(img_path)

    box = async_detect_bound_box(frame)

    if box:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        print(f"Detected box: x={x}, y={y}, w={w}, h={h}")
    else:
        print("No box detected.")
    
    cv2.imshow("Result", resize(frame))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_drone_pos():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(base_dir, "test_images", "drone_test_bg.jpg")
    frame = cv2.imread(img_path)

    frame = resize(frame)

    colors = ("blue", "green")
    blue_pos, green_pos, drone_pos = detect_drone(frame, colors)

    print(f"Blue LED:   {blue_pos}")
    print(f"Green LED:  {green_pos}")
    print(f"Drone center: {drone_pos}")

    overlay = frame.copy()
    draw_led_overlay(overlay, blue_pos, "blue")
    draw_led_overlay(overlay, green_pos, "green")

    if drone_pos is not None:
        cx, cy = drone_pos
        cv2.circle(overlay, (cx, cy), 8, (0, 255, 255), -1)

    out_path = os.path.join(base_dir, "test_images", "drone_test_debug.jpg")
    cv2.imwrite(out_path, overlay)
    print(f"Saved to: {out_path}")

    cv2.imshow("Drone Detection", overlay)
    cv2.waitKey(0)
    cv2.destroyAllWindows()