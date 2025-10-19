from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
import rclpy
from std_msgs.msg import String
from kociemba_utils import solve_cube
from color_detector import get_cube_string, detect_face_colors

UPLOAD_FOLDER = 'uploads'
FRONTEND_FOLDER = '../frontend'  # Path to frontend folder
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# ========================================
# STATIC FILE SERVING FOR FRONTEND
# ========================================
@app.route('/')
def home():
    """Serve the main HTML page from frontend folder"""
    return send_from_directory(FRONTEND_FOLDER, 'index.html')

@app.route('/upload.html')
def upload_page():
    """Serve upload.html if needed"""
    return send_from_directory(FRONTEND_FOLDER, 'upload.html')

@app.route('/js/<path:filename>')
def serve_js(filename):
    """Serve JavaScript files from frontend/js"""
    return send_from_directory(os.path.join(FRONTEND_FOLDER, 'js'), filename)

@app.route('/css/<path:filename>')
def serve_css(filename):
    """Serve CSS files from frontend/css"""
    return send_from_directory(os.path.join(FRONTEND_FOLDER, 'css'), filename)

@app.route('/assets/<path:filename>')
def serve_assets(filename):
    """Serve asset files from frontend/assets"""
    return send_from_directory(os.path.join(FRONTEND_FOLDER, 'assets'), filename)

@app.route('/uploads/<path:filename>')
def serve_uploads(filename):
    """Serve uploaded/detected images"""
    return send_from_directory(UPLOAD_FOLDER, filename)

# ========================================
# EXISTING ROUTES
# ========================================
@app.route('/solve_cube', methods=['POST'])
def solve_cube_route():
    """
    Detect colors → Solve cube → Publish to ROS2
    """
    cube_string = get_cube_string()
    solution = solve_cube(cube_string)

    try:
        rclpy.init()
        from rclpy.node import Node
        node = rclpy.create_node('cube_solution_publisher')
        publisher = node.create_publisher(String, 'cube_solution', 10)
        msg = String()
        msg.data = solution
        publisher.publish(msg)
        node.get_logger().info(f"Published solution: {solution}")
        node.destroy_node()
        rclpy.shutdown()
        return jsonify({'status': 'success', 'solution': solution})
    except Exception as e:
        print("ROS2 publish error:", e)
        return jsonify({'status': 'error', 'message': str(e)})


@app.route('/publish_solution', methods=['POST'])
def publish_solution_route():
    """
    Accept a cube string from Thunder Client and publish its solution to ROS2.
    Example body (JSON):
    {
        "cube_string": "UUUUUUUUURRRRRRBLFFFFLFBFFDDDLDDDRBDLLLLLFDRLBBBFBBRDB"
    }
    """
    data = request.get_json()
    if not data or 'cube_string' not in data:
        return jsonify({'status': 'error', 'message': 'cube_string is required'}), 400

    cube_string = data['cube_string']
    solution = solve_cube(cube_string)

    # Publish to ROS2 topic
    try:
        rclpy.init()
        from rclpy.node import Node
        node = rclpy.create_node('cube_solution_publisher')
        publisher = node.create_publisher(String, 'cube_solution', 10)
        msg = String()
        msg.data = solution
        publisher.publish(msg)
        node.get_logger().info(f"Published solution: {solution}")
        node.destroy_node()
        rclpy.shutdown()

        return jsonify({'status': 'success', 'solution': solution})
    except Exception as e:
        print("ROS2 publish error:", e)
        return jsonify({'status': 'error', 'message': str(e)})


@app.route('/upload_face', methods=['POST'])
def upload_face():
    """
    Upload one face image at a time.
    The client must send:
        - file: the image
        - face: 'U', 'D', 'F', 'B', 'L', 'R'
    """
    if 'file' not in request.files or 'face' not in request.form:
        return jsonify({'status': 'error', 'message': 'Missing file or face'}), 400

    file = request.files['file']
    face = request.form['face'].upper()

    if face not in ['U', 'D', 'F', 'B', 'L', 'R']:
        return jsonify({'status': 'error', 'message': 'Invalid face'}), 400

    filename = f"{face}.jpg"
    filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    file.save(filepath)

    return jsonify({'status': 'success', 'face': face, 'filename': filename})


# ----------------- helper: publish to ROS2 -----------------
def publish_solution(solution: str):
    """
    Helper: publish solution string to the /cube_solution topic via ROS2.
    This initializes and shuts down rclpy per call (safe for the Flask process).
    """
    try:
        rclpy.init()
        from rclpy.node import Node
        node = rclpy.create_node('cube_solution_publisher')
        publisher = node.create_publisher(String, 'cube_solution', 10)
        msg = String()
        msg.data = solution
        publisher.publish(msg)
        node.get_logger().info(f"Published solution: {solution}")
        node.destroy_node()
    finally:
        # ensure shutdown even on exception
        try:
            rclpy.shutdown()
        except Exception:
            pass

# ----------------- GET detected colors for one face -----------------
@app.route('/get_detected_colors', methods=['GET'])
def get_detected_colors():
    """
    Returns detected colors for a requested face as a 3x3 array.
    Query param: ?face=U (one of U,R,F,D,L,B)
    Also writes an annotated image uploads/<face>_detected.jpg (used by frontend)
    Response:
      { "colors": [["U","U","U"],["U","U","U"],["U","U","U"]] }
    """
    face = request.args.get('face', 'U').upper()
    if face not in ['U', 'R', 'F', 'D', 'L', 'B']:
        return jsonify({'error': 'Invalid face param; use U,R,F,D,L,B'}), 400

    image_path = os.path.join(app.config['UPLOAD_FOLDER'], f"{face}.jpg")
    save_path = os.path.join(app.config['UPLOAD_FOLDER'], f"{face}_detected.jpg")

    if not os.path.exists(image_path):
        return jsonify({'error': f'Image {face}.jpg not found in uploads'}), 404

    try:
        # detect_face_colors returns flat list length 9
        colors_flat = detect_face_colors(image_path, save_path=save_path)
        # turn into 3x3
        colors_3x3 = [colors_flat[i:i+3] for i in range(0, 9, 3)]
        return jsonify({'colors': colors_3x3})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

# ----------------- Manual solve (corrected colors or cube_string) -----------------
@app.route('/manual_solve', methods=['POST'])
def manual_solve():
    """
    Accepts either:
    1) a JSON mapping of faces to 3x3 color arrays (expected keys: U,R,F,D,L,B)
       Example:
         {
           "U": [["U","U","U"],["U","U","U"],["U","U","U"]],
           "R": [["R",...],...], ...
         }
    OR
    2) a direct cube_string:
       { "cube_string": "UUU...54 chars..." }

    Returns: { "solution": "<moves>", "status": "success" } or error JSON.
    """
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Missing JSON body'}), 400

    # Option A: direct cube string
    if 'cube_string' in data:
        cube_string = data['cube_string']
        # validate length
        if not isinstance(cube_string, str) or len(cube_string) != 54:
            return jsonify({'error': 'cube_string must be 54-character string'}), 400
        try:
            solution = solve_cube(cube_string)
            # publish
            publish_solution(solution)
            return jsonify({'solution': solution, 'status': 'success'})
        except Exception as e:
            return jsonify({'error': str(e), 'solution': 'Invalid cube state'}), 400

    # Option B: faces mapping
    face_order = ['U', 'R', 'F', 'D', 'L', 'B']
    try:
        # ensure all faces present
        for f in face_order:
            if f not in data:
                return jsonify({'error': f'missing face {f} in JSON'}), 400
            # validate shape
            face_grid = data[f]
            if not (isinstance(face_grid, list) and len(face_grid) == 3):
                return jsonify({'error': f'face {f} must be 3x3 array'}), 400
            for row in face_grid:
                if not (isinstance(row, list) and len(row) == 3):
                    return jsonify({'error': f'face {f} must be 3x3 array'}), 400

        # build cube_string in URFDLB order
        cube_chars = []
        for f in face_order:
            face_grid = data[f]
            for row in face_grid:
                for c in row:
                    if not isinstance(c, str) or len(c) != 1:
                        return jsonify({'error': f'invalid color code in face {f}'}), 400
                    cube_chars.append(c)
        cube_string = ''.join(cube_chars)
        # validate length 54
        if len(cube_string) != 54:
            return jsonify({'error': 'Built cubestr has wrong length'}), 400

        # solve
        solution = solve_cube(cube_string)
        # publish
        publish_solution(solution)
        return jsonify({'solution': solution, 'status': 'success'})

    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    # Use adhoc SSL for HTTPS (allows camera access on local network)
    app.run(host='0.0.0.0', port=5000, debug=True, ssl_context='adhoc')