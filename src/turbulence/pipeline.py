from .cv import *

class CameraPipeline:
    def __init__(self, color: str):
        self.color = color
        self.bbox = None
        self.bbox_center = None
        self.drone_pos = None

    def process(self, frame):
        # we lock box since I assume cam won't move
        if self.bbox is None:
            self.bbox = detect_bound_box(frame)
            if self.bbox is not None:
                self.bbox_center = get_bbox_center(self.bbox)
                print(f"[{self.color}] Bounding box locked! Center: {self.bbox_center}")

        self.drone_pos = detect_drone(frame, self.color)
        frame = draw_overlay(frame, self.bbox, self.drone_pos, self.bbox_center)

        return frame, self.drone_pos, self.bbox, self.bbox_center
    

# not ideal but hardcoded / chnage color depending on cam (green for front, blue for side)
class FrontCamPipeline(CameraPipeline):
    def __init__(self):
        super().__init__(color="green")


class SideCamPipeline(CameraPipeline):
    def __init__(self):
        super().__init__(color="blue")