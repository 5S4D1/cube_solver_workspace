import kociemba

def solve_cube(cube_string: str) -> str:
    """
    Solve a 3x3 cube using the Kociemba algorithm.
    Input:  cube_string in URFDLB order (54 chars)
    Output: move sequence string (e.g. "R U R' U'")
    """
    # Basic validation
    if len(cube_string) != 54:
        raise ValueError("Cube string must have 54 characters")

    try:
        solution = kociemba.solve(cube_string)
        print("Solution:", solution)
        return solution
    except Exception as e:
        print("Error solving cube:", e)
        return "Invalid cube state"
