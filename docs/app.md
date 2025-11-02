# Flask Backend API Documentation (app.py)

**Version:** 1.0  
**Last Updated:** November 2025  
**Author:** Cube Solver Team

---

## Overview

`backend/app.py` is the core Flask web server that orchestrates the Rubik's Cube solving pipeline. It provides:

- **RESTful API endpoints** for cube image uploads, color detection, and solution computation
- **Static file serving** for the frontend HTML/CSS/JS and assets
- **ROS2 integration** to publish solution move sequences to a `/cube_solution` topic
- **Manual override** for correcting detected colors or submitting custom cube states

The application integrates three key components:
1. **Color detection** (`color_detector.py`) — HSV-based sticker recognition
2. **Cube solver** (`kociemba_utils.py`) — Kociemba two-phase algorithm wrapper
3. **ROS2 publisher** — Real-time messaging to a robot/hardware controller

---

## Architecture

```
┌─────────────────┐
│   Frontend      │ (HTML/CSS/JS served by Flask)
│   (Browser)     │
└────────┬────────┘
         │ HTTP/HTTPS
         ▼
┌─────────────────┐
│   Flask App     │ app.py
│   (Port 5000)   │
└────────┬────────┘
         │
         ├─► color_detector.py  (detect colors from images)
         ├─► kociemba_utils.py  (solve cube string)
         └─► ROS2 /cube_solution topic (publish solution)
```

### Key Design Decisions

- **CORS enabled** for all routes — allows frontend to call API from different origins during development.
- **SSL/HTTPS (adhoc)** — required to access camera from browsers on local network (camera API needs secure context).
- **Per-request ROS2 init/shutdown** — Flask is not ROS2-native, so each publish operation initializes and shuts down `rclpy` to avoid threading conflicts.
- **File-based storage** — uploaded images are saved to `backend/uploads/` with standardized filenames (`U.jpg`, `R.jpg`, etc.) to simplify face-by-face detection.

---

## Configuration

### Environment Variables (Optional)

| Variable          | Default         | Description                                  |
|-------------------|-----------------|----------------------------------------------|
| `UPLOAD_FOLDER`   | `uploads`       | Directory for uploaded and annotated images  |
| `FRONTEND_FOLDER` | `../frontend`   | Path to static HTML/CSS/JS frontend files    |

### Flask Configuration

- **Host:** `0.0.0.0` (listens on all interfaces)
- **Port:** `5000`
- **Debug Mode:** `True` (disable in production)
- **SSL Context:** `adhoc` (self-signed cert for HTTPS; install `pyopenssl` if missing)

---

## API Reference

### Static File Serving

#### `GET /`
**Description:** Serve the main landing page.

**Response:** `frontend/index.html`

---

#### `GET /upload.html`
**Description:** Serve the face upload page.

**Response:** `frontend/upload.html`

---

#### `GET /js/<path:filename>`
**Description:** Serve JavaScript files from `frontend/js/`.

**Example:** `/js/main.js`

---

#### `GET /css/<path:filename>`
**Description:** Serve CSS stylesheets from `frontend/css/`.

**Example:** `/css/style.css`

---

#### `GET /assets/<path:filename>`
**Description:** Serve assets (3D models, reports, images) from `frontend/assets/`.

**Example:** `/assets/images/demo.jpg`

---

#### `GET /uploads/<path:filename>`
**Description:** Serve uploaded or detected images from `backend/uploads/`.

**Example:** `/uploads/U_detected.jpg` (annotated detection image)

---

### Cube Solving Workflow

#### `POST /upload_face`
**Description:** Upload a single face image. The frontend should call this endpoint six times (once per face: U, R, F, D, L, B).

**Request:**
- **Content-Type:** `multipart/form-data`
- **Form Fields:**
  - `file` (file): Image file (JPEG/PNG)
  - `face` (string): Face identifier — one of `U`, `R`, `F`, `D`, `L`, `B`

**Response:**
```json
{
  "status": "success",
  "face": "U",
  "filename": "U.jpg"
}
```

**Errors:**
- `400` — Missing `file` or `face`, or invalid face identifier
- `500` — File save error

**Storage:** Image is saved as `backend/uploads/<FACE>.jpg` (overwrites previous uploads).

---

#### `GET /get_detected_colors?face=<FACE>`
**Description:** Detect and return the 3×3 color grid for a specific face. Also generates an annotated image (`<FACE>_detected.jpg`) showing detected colors overlaid on the original.

**Query Parameters:**
- `face` (string, required): Face identifier (`U`, `R`, `F`, `D`, `L`, `B`)

**Response:**
```json
{
  "colors": [
    ["U", "U", "U"],
    ["U", "U", "U"],
    ["U", "U", "U"]
  ]
}
```

**Errors:**
- `400` — Invalid face parameter
- `404` — Image `<FACE>.jpg` not found in `uploads/`
- `500` — Color detection error

**Side Effects:** Writes `backend/uploads/<FACE>_detected.jpg` (annotated image used by frontend preview).

**Implementation Details:**
- Calls `detect_face_colors(image_path, save_path)` from `color_detector.py`
- Returns a 3×3 nested list (row-major order)
- Each color is a single-character Kociemba notation letter: `U`, `R`, `F`, `D`, `L`, `B`, or `X` (unknown)

---

#### `POST /solve_cube`
**Description:** Execute the full pipeline: detect all six faces → assemble cube string → solve → publish to ROS2.

**Request:**
- **Content-Type:** `application/json` (body can be empty)
- **Precondition:** All six face images (`U.jpg`, `R.jpg`, `F.jpg`, `D.jpg`, `L.jpg`, `B.jpg`) must exist in `backend/uploads/`.

**Response (Success):**
```json
{
  "status": "success",
  "solution": "R U R' U' F2 D B' ..."
}
```

**Response (Error):**
```json
{
  "status": "error",
  "message": "Invalid cube state"
}
```

**Flow:**
1. Call `get_cube_string()` → detects colors from all six images and assembles 54-character cube string
2. Call `solve_cube(cube_string)` → computes solution using Kociemba algorithm
3. Publish solution to ROS2 topic `/cube_solution` (message type: `std_msgs/String`)
4. Return solution to client

**Errors:**
- `500` — Missing images, invalid cube state, or ROS2 publish failure

**Notes:**
- This endpoint initializes and shuts down `rclpy` per request (safe for Flask single-threaded mode).
- If ROS2 is not running or the topic has no subscribers, publish still succeeds (fire-and-forget).

---

#### `POST /publish_solution`
**Description:** Accept a pre-computed cube string, solve it, and publish the solution to ROS2. Useful for testing or when cube state is provided manually.

**Request:**
```json
{
  "cube_string": "UUUUUUUUURRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"
}
```

**Response (Success):**
```json
{
  "status": "success",
  "solution": "R U R' U' ..."
}
```

**Response (Error):**
```json
{
  "status": "error",
  "message": "cube_string is required"
}
```

**Validation:**
- `cube_string` must be exactly 54 characters
- Must represent a valid, solvable cube state (Kociemba will raise exception otherwise)

**Flow:**
1. Extract `cube_string` from request JSON
2. Call `solve_cube(cube_string)`
3. Publish solution to `/cube_solution` ROS2 topic
4. Return solution

---

#### `POST /manual_solve`
**Description:** Advanced endpoint for submitting corrected or manually entered cube states. Accepts either a direct 54-character cube string OR a structured JSON object with per-face 3×3 grids.

**Request Format A (Direct Cube String):**
```json
{
  "cube_string": "UUUUUUUUURRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB"
}
```

**Request Format B (Per-Face Grids):**
```json
{
  "U": [["U","U","U"], ["U","U","U"], ["U","U","U"]],
  "R": [["R","R","R"], ["R","R","R"], ["R","R","R"]],
  "F": [["F","F","F"], ["F","F","F"], ["F","F","F"]],
  "D": [["D","D","D"], ["D","D","D"], ["D","D","D"]],
  "L": [["L","L","L"], ["L","L","L"], ["L","L","L"]],
  "B": [["B","B","B"], ["B","B","B"], ["B","B","B"]]
}
```

**Response (Success):**
```json
{
  "status": "success",
  "solution": "R U R' U' F2 ..."
}
```

**Response (Error):**
```json
{
  "error": "missing face U in JSON",
  "solution": "Invalid cube state"
}
```

**Validation:**
- Format A: `cube_string` must be 54 characters
- Format B: All six faces (`U`, `R`, `F`, `D`, `L`, `B`) must be present; each must be a 3×3 array of single-character strings

**Flow:**
1. Parse JSON and construct 54-character cube string
2. Call `solve_cube(cube_string)`
3. Call `publish_solution(solution)` → publishes to ROS2
4. Return solution

**Use Cases:**
- Frontend "manual correction" UI where user edits detected colors
- Testing with known cube states
- Integration with external color detection systems

---

## Helper Functions

### `publish_solution(solution: str)`
**Description:** Internal helper to publish a solution string to the ROS2 `/cube_solution` topic.

**Parameters:**
- `solution` (str): Move sequence (e.g., `"R U R' U'"`)

**Behavior:**
- Initializes `rclpy`
- Creates a temporary node `cube_solution_publisher`
- Publishes a `std_msgs/String` message with `data=solution`
- Shuts down `rclpy` (ensures cleanup even on exception)

**Thread Safety:** This pattern is safe for Flask's single-threaded default mode. For production with threading/Gunicorn, consider a persistent ROS2 node in a separate thread or process.

---

## Deployment

### Local Development

1. **Install dependencies:**
   ```bash
   cd backend
   pip install flask flask-cors kociemba opencv-python numpy pyopenssl rclpy std_msgs
   ```

2. **Prepare upload directory:**
   ```bash
   mkdir -p uploads
   ```

3. **Run the server:**
   ```bash
   python app.py
   ```

   Server starts on `https://0.0.0.0:5000` (HTTPS with self-signed cert).

4. **Access frontend:**
   Open `https://<your-ip>:5000/` in a browser. Accept the self-signed certificate warning.

### Production Considerations

- **Disable debug mode:** Set `debug=False` in `app.run()`
- **Use a production WSGI server:** Deploy with Gunicorn or uWSGI instead of Flask's built-in server
  ```bash
  gunicorn --bind 0.0.0.0:5000 --certfile cert.pem --keyfile key.pem app:app
  ```
- **Persistent ROS2 node:** Initialize `rclpy` once at startup and use a background thread/process for publishing to avoid overhead
- **Error handling & logging:** Replace `print()` with proper logging (`logging` module) and add structured error responses
- **Authentication:** Add token-based auth if exposing to public networks
- **Rate limiting:** Use Flask-Limiter to prevent abuse on upload/solve endpoints

### ROS2 Setup

Ensure ROS2 is installed and sourced before running `app.py`:

```bash
source /opt/ros/<distro>/setup.bash  # e.g., humble, foxy
```

Verify the `/cube_solution` topic:
```bash
ros2 topic echo /cube_solution
```

---

## Error Handling

| HTTP Status | Meaning                                   | Common Causes                              |
|-------------|-------------------------------------------|--------------------------------------------|
| `200`       | Success                                   | Operation completed successfully           |
| `400`       | Bad Request                               | Missing parameters, invalid JSON, bad face |
| `404`       | Not Found                                 | Image not uploaded yet                     |
| `500`       | Internal Server Error                     | Color detection failure, invalid cube      |

**Error Response Format:**
```json
{
  "status": "error",
  "message": "Detailed error message",
  "error": "Alternative error field (some endpoints)"
}
```

---

## Example Workflows

### Workflow 1: Full Automated Pipeline

1. User uploads six face images via `POST /upload_face` (U, R, F, D, L, B)
2. Frontend displays detected colors using `GET /get_detected_colors?face=U` (repeat for all faces)
3. User confirms and clicks "Solve"
4. Frontend calls `POST /solve_cube`
5. Backend detects colors, solves, publishes to ROS2, and returns solution
6. Frontend displays solution moves

### Workflow 2: Manual Correction

1. User uploads images and reviews detected colors
2. Frontend detects errors in color detection (e.g., red detected as orange)
3. User edits colors in a correction UI
4. Frontend submits corrected per-face grids to `POST /manual_solve`
5. Backend assembles cube string, solves, publishes, returns solution

### Workflow 3: Direct Testing (Thunder Client / Postman)

1. Prepare a known cube string (54 chars)
2. Send `POST /publish_solution` with JSON: `{"cube_string": "..."}`
3. Observe solution in response and on ROS2 topic `/cube_solution`

---

## Testing

### Unit Tests

Create `backend/test_app.py`:

```python
import pytest
from app import app

@pytest.fixture
def client():
    app.config['TESTING'] = True
    with app.test_client() as client:
        yield client

def test_home(client):
    rv = client.get('/')
    assert rv.status_code == 200

def test_upload_face_missing_params(client):
    rv = client.post('/upload_face')
    assert rv.status_code == 400
    assert b'Missing file or face' in rv.data
```

Run:
```bash
pytest backend/test_app.py
```

### Integration Tests

- Upload test cube images and verify solution matches expected output
- Mock ROS2 publisher to verify messages are published with correct content

---

## Security Notes

- **HTTPS Required:** Camera access requires HTTPS. The server uses `ssl_context='adhoc'` for local dev. Use proper certificates in production.
- **File Upload Validation:** Currently allows any file type. Add MIME type checks and file size limits.
- **Input Sanitization:** Validate `face` parameter and `cube_string` length/format to prevent injection or DoS.
- **CORS Configuration:** In production, restrict CORS to specific origins instead of allowing all (`CORS(app, origins=['https://yourdomain.com'])`).

---

## Troubleshooting

### Issue: "Cannot read image" error

**Cause:** Image file not found or corrupted.

**Solution:** Verify all six face images exist in `backend/uploads/` with correct names (`U.jpg`, `R.jpg`, etc.).

---

### Issue: ROS2 publish fails

**Cause:** ROS2 not sourced or `rclpy` not installed.

**Solution:**
```bash
source /opt/ros/<distro>/setup.bash
pip install rclpy
```

---

### Issue: "Invalid cube state" from solver

**Cause:** Detected colors do not represent a valid, solvable cube (e.g., wrong number of each color, impossible corner/edge positions).

**Solution:**
- Review detected colors using `/get_detected_colors` and `<FACE>_detected.jpg` images
- Use `/manual_solve` to correct misdetected colors
- Improve lighting and image quality during capture

---

### Issue: HTTPS certificate warnings

**Cause:** Self-signed certificate used for local dev.

**Solution:** Accept the warning in browser, or generate a trusted cert using Let's Encrypt or mkcert for local dev.

---

## Related Documentation

- [color_detector.py](./color_detector.md) — Color detection implementation and HSV tuning
- [kociemba_utils.py](./kociemba_utils.md) — Cube solver wrapper
- [cube_solver_node.md](./cube_solver_node.md) — ROS2 node that consumes `/cube_solution` topic

---

## Changelog

| Version | Date       | Changes                                           |
|---------|------------|---------------------------------------------------|
| 1.0     | Nov 2025   | Initial release with full pipeline and ROS2 pub   |

---

## Contributing

When adding new endpoints:
1. Add route handler with docstring describing purpose, request/response formats, and errors
2. Update this documentation with new API reference entry
3. Add unit tests in `backend/test_app.py`
4. Consider logging and error handling for production readiness

---

**Questions or Issues?** Open an issue on the project repository or contact the development team.
