from color_detector import get_cube_string
from kociemba_utils import solve_cube

# 1. Detect cube state
cube_str = get_cube_string()
print("Detected cube string:", cube_str)

# 2. Solve
solution = solve_cube(cube_str)
print("Solution moves:", solution)
