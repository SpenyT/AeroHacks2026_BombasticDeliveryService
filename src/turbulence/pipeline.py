from .cv import *

class CameraPipeline:
    def __init__(self, bbox, bbox_center, color: tuple[str, str]):
        self.color = color
        self.drone_pos = None
        self.led1_pos = None
        self.led2_pos = None
        self.bbox = bbox
        self.bbox_center = bbox_center
        

    def process(self, frame):
        result = detect_drone(frame, self.color)
        if result is not None:
            self.led1_pos, self.led2_pos, self.drone_pos = result
        else:
            self.led1_pos = self.led2_pos = self.drone_pos = None

        frame = draw_overlay(frame, self.bbox, self.drone_pos, self.bbox_center)
        return frame, self.drone_pos