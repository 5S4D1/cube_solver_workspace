# color_detector.py Documentation

This module contains utilities to detect sticker colours from photos of a 3x3 Rubik's Cube and to assemble those detections into a cube state string compatible with the Kociemba solver.

The implementation is tuned for cubes with a dark/black plastic base and colored stickers, and uses simple HSV thresholds + median sampling to be more robust to highlights and camera noise.

---

## Quick overview

- Main public functions
  - `detect_face_colors(image_path, save_path=None) -> List[str]` — detect the 9 stickers on a single face and optionally write an annotated image with rectangles drawn around sampled stickers.
  - `get_cube_string(upload_folder='uploads') -> dict` — reads six images (U.jpg, R.jpg, F.jpg, D.jpg, L.jpg, B.jpg) from `upload_folder`, annotates and saves detected images (e.g. `U_detected.jpg`) and returns a structured result with per-face detections, an `ok` flag, and any `errors`.

- Helper functions (smaller building blocks)
  - `classify_color(hsv_pixel) -> str` — classifies a single HSV pixel tuple into Kociemba letters `'U','R','F','D','L','B'` or `'X'` for unknown/invalid.
  - `letter_to_bgr(letter) -> Tuple[int,int,int]` — maps Kociemba letters to BGR tuples used when drawing rectangles on annotated images.
  - `in_range(hsv_pixel, hsv_range) -> bool` — low-level helper to compare HSV triplet against a (lower, upper) bound.

---

## Installation / dependencies

color_detector uses OpenCV and NumPy. Install them using pip:

```bash
pip install opencv-python numpy
```

If your project uses `backend/requirements.txt`, make sure it lists `opencv-python` and `numpy`.

---

## Details & behavior

- Input normalization:
  - Input images are read with `cv2.imread()` and resized to 300x300 pixels before processing. This normalizes coordinates and sampling.

- Sticker sampling:
  - Each 3x3 sticker grid cell is sampled at its center. A 10x10 region around the center is taken and the median HSV values are used to reduce specular highlights and outlier pixels.

- HSV-based detection:
  - The module contains a set of tuned HSV ranges (`HSV_RANGES`) for white, yellow, red (split into two hue ranges to handle hue wrap), orange, green, blue, and black (the cube plastic).
  - `classify_color` implements a decision tree combining explicit range checks, heuristic fallbacks, and tie-breakers when pixels match both red and orange ranges. Very dark pixels (low V) are considered `'X'` unknown/invalid (likely cube base or background).

  - Logging: The module uses Python's `logging` module. Debug/diagnostic messages that were previously printed are now emitted via a module-level logger (`logging.getLogger(__name__)`) at `DEBUG` level. When run as a script the final cube string is logged at `INFO` level. Do not configure logging inside this library; configure it in the application or test runner. Example to enable debug output during development:

  ```py
  import logging
  logging.basicConfig(level=logging.DEBUG)
  # or target the module specifically
  logging.getLogger('backend.color_detector').setLevel(logging.DEBUG)
  ```

- Output format:
  - `detect_face_colors` returns a list of 9 single-character strings (letters in Kociemba notation or `'X'`). The order is row-major (top-left to bottom-right).
  - `get_cube_string` returns a dictionary with the following keys:
    - `ok` (bool): True when all six faces were detected successfully.
    - `cube_string` (str|null): the 54-character cube string in Kociemba order `U R F D L B` when `ok` is True, otherwise `null`.
    - `faces` (dict): mapping from face id (`U`,`R`,`F`,`D`,`L`,`B`) to a list of 9 detected letters or `null` if detection failed for that face.
    - `errors` (list): list of per-face error objects with fields `face`, `image`, and `error` (message).

---

## Function reference and examples

detect_face_colors

- Signature: `detect_face_colors(image_path, save_path=None) -> List[str]`
- Input:
  - `image_path` (str): path to a single face image (expects a 3x3 face roughly filling the image after resizing to 300x300).
  - `save_path` (str|None): optional path to write an annotated image showing rectangles and detected colors.
- Output: list of 9 letters, each one of `['U','D','L','R','F','B','X']` (`'X'` = unknown/invalid).
- Errors: raises `ValueError` if the image cannot be read.

Example:

```py
from backend.color_detector import detect_face_colors

colors = detect_face_colors('backend/uploads/U.jpg', save_path='backend/uploads/U_detected.jpg')
print(colors)  # ['U','U','U','U','U','U','U','U','U'] for a solved white face
```

get_cube_string

 - Signature: `get_cube_string(upload_folder='uploads') -> dict`
 - Behavior: looks for files named `U.jpg`, `R.jpg`, `F.jpg`, `D.jpg`, `L.jpg`, `B.jpg` inside `upload_folder`, calls `detect_face_colors` for each, writes annotated `*_detected.jpg` files alongside them, and returns a structured result describing per-face detections and any errors.
 - Errors: per-face errors are captured and returned in the `errors` list rather than being raised; the caller should check the `ok` flag and `errors` to decide whether to retry capturing specific faces.

Example:

```py
from backend.color_detector import get_cube_string

result = get_cube_string('backend/uploads')
if result['ok']:
  print(result['cube_string'])  # 54-character string in URFDLB order
else:
  print('Detection failed for faces:', [e['face'] for e in result['errors']])
```

Note: The function currently hard-codes the `upload_folder` and expects exact filenames; caller code should ensure images exist and are named correctly.

---

## Error modes and diagnostics

- Image read failure: `cv2.imread()` returns `None` -> `ValueError` with a message about not being able to read the image.
 - Unknown/invalid sticker: classifier returns `'X'`. The module emits debug logs when encountering `'X'` or ambiguous red/orange colors. Configure the Python logging system in your application to view these diagnostics (see the example above).

---

## Edge cases, limitations, and recommendations

- Hard-coded sizes and coordinates: the current implementation resizes images to 300x300 and samples centers using fixed offsets. This works well when input images contain a roughly centered face filling most of the frame. For more robust operation, consider detecting the face region dynamically (e.g., using contours or a simple grid detection), or allow the caller to pass pre-aligned/cropped faces.

- Threshold tuning: HSV ranges are tuned for a specific camera lighting and a dark cube; they may require adjustments for different cameras, lighting, or sticker materials. Consider externalizing these ranges to a JSON/YAML config or exposing them as function parameters.

- Ambiguous red vs orange: the code has heuristics (saturation thresholds and hue distance) to break ties. You may still get misclassifications; adding a small calibration step (ask user to provide a single reference image) can dramatically increase accuracy.

- Unknown `'X'`: currently propagated into the 54-char cube string, which will make the Kociemba solver fail. Prefer returning a structured result with a success flag and per-face diagnostics so the caller can request a re-capture in case of `'X'` values.

---

## Tests to add

- Unit tests for `classify_color` with synthetic HSV values covering red/orange tie cases, low-V (black), white (low saturation/high V), and edge hue values.
- Integration tests using the sample images in `backend/uploads` to verify `detect_face_colors` and `get_cube_string` produce expected output. Add these to `backend/test_solver.py` or create a dedicated `test_color_detector.py`.


