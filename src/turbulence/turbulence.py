import numpy as np
import asyncio
import cv2
import os
from .video_input import video_input
from .pipeline import CameraPipeline
from .cv import *

DISPLAY_HEIGHT = 480

# options
SIDE_CAM_INDEX  = 0
FRONT_CAM_INDEX = 1
SIDE_CAM_COLORS  = ("", "")
FRONT_CAM_COLORS = ("", "")

TEST_MODE = True

# FOR TESTING MODE
LEFT_BBOX         = ()
LEFT_BBOX_CENTER  = ()
FRONT_BBOX        = ()
FRONT_BBOX_CENTER = ()


def get_img(img_name: str):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(base_dir, "test_images", img_name)
    return cv2.imread(img_path)

async def try_get_bbox(front_frame, side_frame, max_retries=3):
    front_bbox = None
    side_bbox = None

    for attempt in range(max_retries):
        print(f"Requesting bounding boxes... (attempt {attempt + 1}/{max_retries})")
        
        results = await asyncio.gather(
            async_detect_bound_box(front_frame.copy()) if front_bbox is None else asyncio.sleep(0, result=front_bbox),
            async_detect_bound_box(side_frame.copy()) if side_bbox is None else asyncio.sleep(0, result=side_bbox),
        )
        front_bbox, side_bbox = results

        if front_bbox is not None and side_bbox is not None:
            break

        if front_bbox is None:
            print("- Front bbox failed.")
        if side_bbox is None:
            print("- Side bbox failed.")

    return front_bbox, side_bbox


async def run(test_mode=False):
    if not test_mode:
        front_cam, side_cam = video_input()

    front_frame = resize(get_img("bb_test_1.jpg") if test_mode else front_cam.read())
    side_frame = resize(get_img("bb_test_2.jpg") if test_mode else side_cam.read())

    front_bbox, side_bbox = await try_get_bbox(front_frame, side_frame)

    front_pipeline = CameraPipeline(front_bbox, get_bbox_center(front_bbox), FRONT_CAM_COLORS)
    side_pipeline  = CameraPipeline(side_bbox,  get_bbox_center(side_bbox),  SIDE_CAM_COLORS )

    print(f"Running {'(TEST MODE)' if test_mode else ''}... Press 'q' to quit")

    while True:
        front_frame = resize(get_img("bb_test_1.jpg")) if TEST_MODE else resize(front_cam.read())
        side_frame  = resize(get_img("bb_test_2.jpg")) if TEST_MODE else resize(side_cam.read())

        front_frame, front_drone = front_pipeline.process(front_frame)
        side_frame, side_drone = side_pipeline.process(side_frame)

        # Label feeds
        cv2.putText(front_frame, "FRONT", (10, front_frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(side_frame, "SIDE", (10, side_frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # honestly, me and the hommies hate the padding
        combined = np.hstack([front_frame, side_frame])
        cv2.imshow("Drone Tracker", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if not test_mode:
        front_cam.release()
        side_cam.release()
    cv2.destroyAllWindows()


def turbulence():
    print("Initiating Turbulence Dampening Algorithm...")
    asyncio.run(run(TEST_MODE))
    
