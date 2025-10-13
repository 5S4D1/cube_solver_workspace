import cv2
import numpy as np
import os

# Map colors to Kociemba notation
COLOR_MAP = {
    'white': 'U',
    'yellow': 'D',
    'red': 'L',
    'orange': 'R',
    'green': 'F',
    'blue': 'B'
}

# HSV color ranges for basic cube colors - adjusted for black cube with colored stickers
HSV_RANGES = {
    'white':   ((0, 0, 120), (180, 50, 255)),      # lower brightness threshold, higher saturation tolerance
    'yellow':  ((20, 50, 80), (35, 255, 255)),     # lower saturation and brightness for stickers on black
    'red1':    ((0, 120, 120), (8, 255, 255)),     # narrower red range, higher saturation
    'red2':    ((165, 120, 120), (180, 255, 255)), # narrower red range, higher saturation  
    'orange':  ((8, 100, 100), (25, 255, 255)),    # wider orange range, starts after red1
    'green':   ((40, 50, 50), (85, 255, 255)),
    'blue':    ((90, 50, 50), (130, 255, 255)),
    'black':   ((0, 0, 0), (180, 255, 80))         # add black detection for cube base
}

def detect_face_colors(image_path, save_path=None):
    """
    Detect the 9 colors of a single face and optionally save annotated image.
    Returns a list of color letters in row-major order.
    """
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Cannot read image {image_path}")
    img = cv2.resize(img, (300, 300))  # normalize size
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    colors = []
    square_size = 100

    for row in range(3):
        for col in range(3):
            # center coordinates of the sticker
            cx = col * 100 + 50
            cy = row * 100 + 50
            # sample a 10x10 ROI around center (clamp to image bounds)
            y1 = max(cy - 5, 0)
            y2 = min(cy + 5, hsv.shape[0])
            x1 = max(cx - 5, 0)
            x2 = min(cx + 5, hsv.shape[1])
            roi = hsv[y1:y2, x1:x2]
            if roi.size == 0:
                hsv_pixel = tuple(int(v) for v in hsv[cy, cx])
            else:
                # use median to reduce influence of specular highlights/outliers
                h_med = int(np.median(roi[:, :, 0]))
                s_med = int(np.median(roi[:, :, 1]))
                v_med = int(np.median(roi[:, :, 2]))
                hsv_pixel = (h_med, s_med, v_med)
            color_letter = classify_color(hsv_pixel)
            colors.append(color_letter)
            
            # Debug output for problematic colors (white, yellow, unknown)
            if color_letter in ['U', 'D', 'X']:
                color_name = {'U': 'white', 'D': 'yellow', 'X': 'unknown/black'}[color_letter]
                print(f"Sticker at row {row}, col {col}: HSV={hsv_pixel} -> {color_letter} ({color_name})")
            
            # Debug output for red/orange classification
            elif color_letter in ['L', 'R']:
                print(f"Sticker at row {row}, col {col}: HSV={hsv_pixel} -> {color_letter} ({'red' if color_letter == 'L' else 'orange'})")

            # Draw rectangle for visualization
            top_left = (col * 100 + 10, row * 100 + 10)
            bottom_right = (col * 100 + 90, row * 100 + 90)
            color_bgr = letter_to_bgr(color_letter)
            cv2.rectangle(img, top_left, bottom_right, color_bgr, 2)

    if save_path:
        cv2.imwrite(save_path, img)

    return colors

def letter_to_bgr(letter):
    """Map Kociemba letter to BGR color for drawing"""
    mapping = {
        'U': (255, 255, 255),  # white
        'D': (0, 255, 255),    # yellow
        'L': (0, 0, 255),      # red
        'R': (0, 165, 255),    # orange
        'F': (0, 255, 0),      # green
        'B': (255, 0, 0),      # blue
        'X': (0, 0, 0)         # unknown
    }
    return mapping.get(letter, (0, 0, 0))


def classify_color(hsv_pixel):
    h, s, v = hsv_pixel
    
    # Black detection first (very low brightness)
    if v < 60:  # very dark pixels are likely black cube base, not stickers
        return 'X'  # treat as unknown/invalid
    
    # White detection - more tolerant for stickers on black cube
    if v > 120 and s < 50:  # bright with low saturation = white sticker
        return 'U'
    
    # Yellow detection - more tolerant for stickers on black background  
    elif in_range(hsv_pixel, HSV_RANGES['yellow']):
        return 'D'
    
    # Additional yellow check for edge cases (sometimes appears desaturated on black)
    elif 18 <= h <= 40 and v > 100 and s > 30:  # yellow-ish hue with reasonable brightness
        return 'D'
    
    # Red detection - check both ranges for hue wrapping
    red1_match = in_range(hsv_pixel, HSV_RANGES['red1'])
    red2_match = in_range(hsv_pixel, HSV_RANGES['red2'])
    orange_match = in_range(hsv_pixel, HSV_RANGES['orange'])
    
    # If clearly red (either range) and not orange
    if (red1_match or red2_match) and not orange_match:
        return 'L'
    
    # If clearly orange and not red
    if orange_match and not red1_match and not red2_match:
        return 'R'
    
    # If both red and orange match (edge case), use hue distance + saturation
    if orange_match and (red1_match or red2_match):
        # Higher saturation usually indicates more vivid red vs orange
        if s > 180:  # very saturated - likely red
            return 'L'
        elif s < 140:  # less saturated - likely orange  
            return 'R'
        else:
            # Use hue distance as tiebreaker
            dist_to_red = min(abs(h - 0), abs(h - 180))
            dist_to_orange = abs(h - 15)
            return 'L' if dist_to_red < dist_to_orange else 'R'
    
    # Green detection
    elif in_range(hsv_pixel, HSV_RANGES['green']):
        return 'F'
    
    # Blue detection  
    elif in_range(hsv_pixel, HSV_RANGES['blue']):
        return 'B'
    
    else:
        return 'X'  # unknown

def in_range(hsv_pixel, hsv_range):
    lower, upper = hsv_range
    return all(lower[i] <= hsv_pixel[i] <= upper[i] for i in range(3))

def get_cube_string(upload_folder='uploads'):
    """
    Process all 6 faces and return a cube string in Kociemba format.
    Also saves annotated images showing detected colors.
    Order: U, R, F, D, L, B
    """
    face_order = ['U', 'R', 'F', 'D', 'L', 'B']
    cube_string = ''
    
    for face_id in face_order:
        image_path = os.path.join(upload_folder, f"{face_id}.jpg")
        save_path = os.path.join(upload_folder, f"{face_id}_detected.jpg")  # save annotated image
        
        # Detect colors and save annotated image
        colors = detect_face_colors(image_path, save_path=save_path)
        cube_string += ''.join(colors)
    
    # print("Cube String:", cube_string)
    return cube_string


if __name__ == "__main__":
    cube_str = get_cube_string()
    print("Cube String:", cube_str)
