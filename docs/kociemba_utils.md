# kociemba_utils.py Documentation

This module provides a thin wrapper around the `kociemba` package to solve a 3x3 Rubik's Cube state string using the Kociemba two-phase algorithm.

## Purpose

`kociemba_utils.py` exposes a single convenience function `solve_cube` that:
- Validates a cube string is 54 characters long
- Calls `kociemba.solve(...)` to compute a solution
- Prints and returns the solution string

## Installation

The project depends on the `kociemba` Python package. Install it in your environment:

```bash
pip install kociemba
```

If you install dependencies from `backend/requirements.txt`, ensure `kociemba` is listed there.

## Function API

```py
from kociemba_utils import solve_cube

solution = solve_cube(cube_string)
```

Function: solve_cube(cube_string: str) -> str

- Parameters:
  - `cube_string` -- A 54-character string representing the cube facelets in the Kociemba order (URFDLB). Each character represents the face colour of a sticker (commonly 'U', 'R', 'F', 'D', 'L', 'B').

- Returns:
  - On success: a move-sequence string returned by `kociemba.solve`, e.g. "R U R' U'".
  - On error: the function catches exceptions from `kociemba.solve`, prints the error, and returns the string "Invalid cube state".

- Raises:
  - `ValueError` if `cube_string` is not exactly 54 characters long.

## Cube string format (URFDLB)

Kociemba expects the cube string in the face order: U, R, F, D, L, B — each face provided as 9 characters (row-major). Example solved-cube input:

```
UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB
```

Make sure the characters represent a valid cube colouring and that the string uses a consistent character set (e.g. 'U', 'R', 'F', 'D', 'L', 'B' or colour initials). If the colouring/order is wrong, `kociemba.solve` will raise an exception.

## Example usage

```py
from backend.kociemba_utils import solve_cube

# Example: solved cube
solved = 'UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB'
print(solve_cube(solved))  # -> typically an empty string or "" (no moves needed)

# Example: invalid length
try:
    solve_cube('too short')
except ValueError as e:
    print(e)
```

### Error handling and suggestions

- The current implementation prints errors and returns a sentinel string `"Invalid cube state"`. Calling code should check for that return value.
- For cleaner error handling in production, consider re-raising the original exception or returning a structured result (e.g., tuple (success: bool, solution_or_error: str)).
- Add logging instead of printing for better observability.

### Edge cases and notes

- Non-54 length: `ValueError` is raised.
- Syntactically correct but impossible cube states: `kociemba.solve` will raise; the wrapper returns "Invalid cube state".
- Input character set: be consistent (don't mix colour names and face letters).

### Tests

There is a `backend/test_solver.py` in this project — extend it with tests covering:
- Correct solved-cube input (expect empty solution or no-move result)
- A known scrambled cube with a determinable solution
- Bad input length -> assert ValueError
- Invalid but 54-length input -> assert the wrapper returns "Invalid cube state" (or update wrapper to raise)


