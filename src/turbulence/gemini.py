import cv2
import json
import os
from google import genai
from google.genai import types


_MODEL = "gemini-2.5-flash"
_client = None


def _get_client():
    global _client
    if _client is None:
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("No API key. Set GEMINI_API_KEY env var.")
        _client = genai.Client(api_key=api_key)
    return _client


def _frame_to_bytes(frame) -> bytes:
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return buf.tobytes()


_BBOX_SCHEMA = types.Schema(
    type=types.Type.OBJECT,
    properties={
        "y_min": types.Schema(type=types.Type.INTEGER),
        "x_min": types.Schema(type=types.Type.INTEGER),
        "y_max": types.Schema(type=types.Type.INTEGER),
        "x_max": types.Schema(type=types.Type.INTEGER),
    },
    required=["y_min", "x_min", "y_max", "x_max"],
)


def request_bound_box(frame) -> tuple:
    img_h, img_w = frame.shape[:2]

    prompt = (
        f"This image is {img_w}x{img_h} pixels. It shows a wooden-framed cube structure "
        "with clear plastic sheeting stapled to the frame. "
        "Detect the back panel opening — the rectangular open area on the back face of the enclosure, "
        "bounded above by the top horizontal wood rail and below by the plywood base. "
        "Return the bounding box coordinates normalized to 0-1000."
    )

    client = _get_client()
    image_part = types.Part.from_bytes(data=_frame_to_bytes(frame), mime_type="image/jpeg")
    response = client.models.generate_content(
        model=_MODEL,
        contents=[image_part, prompt],
        config=types.GenerateContentConfig(
            temperature=0.0,
            response_mime_type="application/json",
            response_schema=_BBOX_SCHEMA,
        ),
    )

    data = json.loads(response.text)
    y_min = data["y_min"]
    x_min = data["x_min"]
    y_max = data["y_max"]
    x_max = data["x_max"]

    x = int(x_min / 1000 * img_w)
    y = int(y_min / 1000 * img_h)
    bw = int((x_max - x_min) / 1000 * img_w)
    bh = int((y_max - y_min) / 1000 * img_h)

    if bw <= 0 or bh <= 0:
        raise RuntimeError(f"Invalid box dimensions: {bw}x{bh}")

    return (x, y, bw, bh)
