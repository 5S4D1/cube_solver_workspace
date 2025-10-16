# Cube Solver Node Documentation

This document describes the `cube_solver_node` from the `cube_solver` ROS 2 package. It covers the node's responsibilities, runtime behavior, public interfaces (topics and parameters), message flow, error handling, deployment, and operational guidance.

## Overview

`cube_solver_node` bridges a high-level Rubik’s Cube solution (as a sequence of moves) to a hardware controller (ESP32) over a serial connection. It subscribes to a ROS 2 topic that provides the full solution string (e.g., `"R U R' U'"`), relays each move to the ESP32 one-by-one, and waits for an acknowledgment ("DONE") after each move before proceeding. It also publishes each move on a ROS topic for UI/telemetry purposes.

Key capabilities:
- Receive a full solution string via ROS 2
- Parse the solution into individual moves
- Send moves over serial to ESP32 at 115200 baud
- Wait for a per-move acknowledgment with timeout
- Publish each move for dashboards/monitoring
- Log all significant events and error conditions

## Node Identity

- Package: `cube_solver`
- Executable (console script): `cube_solver_node`
- Node name: `cube_solver_node`

## Runtime Dependencies

- ROS 2 (rclpy)
- `std_msgs` (String)
- Python packages: `pyserial`, `time` (stdlib)
- Hardware: ESP32 connected at `/dev/ttyUSB0`

Ensure `pyserial` is installed in the environment. In this repository, Python dependencies for other components live under `backend/requirements.txt`; add `pyserial` there if not present when building a single consolidated environment.

## Topics

- Subscription:
  - Name: `cube_solution`
  - Type: `std_msgs/msg/String`
  - Description: Provides the full cube solution as a space-separated string of moves. Example: `"R U R' U' F2 D"`.

- Publication:
  - Name: `cube_move`
  - Type: `std_msgs/msg/String`
  - Description: Emits each move that is being sent to the ESP32, one message per move. Useful for dashboards or other consumers that visualize progress in real time.

## Serial Protocol

- Port: `/dev/ttyUSB0`
- Baud rate: `115200`
- Timeout (read): `1` second per `serial.Serial` configuration, with an overall per-move wait loop of up to `10` seconds for acknowledgment.
- Outbound message: the move string followed by a newline (`\n`). Example: `b"R\n"`.
- Inbound acknowledgment: the literal string `"DONE"` (without surrounding quotes) indicates successful execution of the move. Any other text is logged at `INFO` level.

If the node cannot open the serial port at startup, it logs an error and continues running; move sending will be skipped with a warning until a serial connection is available (requires restarting the node after fixing the connection).

## Behavior and Flow

1. On startup, the node attempts to open `/dev/ttyUSB0` at 115200 baud. Success is logged. Failure is logged and the node continues, but cannot send moves.
2. When a `cube_solution` message arrives:
   - The string is trimmed and split on whitespace into a list of moves (e.g., `["R", "U", "R'", "U'"]`).
   - For each move in order:
     - The node writes the move to the serial port with a trailing newline.
     - It publishes the same move on `cube_move` for downstream consumers.
     - It polls the serial input buffer for up to 10 seconds. If a line equals `DONE`, the next move is sent. Any other lines are logged. If `DONE` is not received within the timeout, the sequence is aborted.
   - A summary is logged indicating success or early termination.

## Timeout and Error Handling

- Serial connect failure: Logged as an error at startup; the node continues running. Subsequent move attempts immediately warn and return failure.
- Per-move timeout: If `DONE` is not received within 10 seconds after sending a move, the node logs an error and aborts the remaining sequence.
- Serial exceptions: Any exception during write/read is caught, logged as an error, and treated as a failed move, aborting the sequence.
- Keyboard interrupt: Gracefully shuts down, closing the serial port if open.

## Input/Output Contract

- Input: `std_msgs/String` solution where `data` is a space-separated sequence of cube moves expressed in your robot’s move vocabulary. Common tokens might include: `R, L, U, D, F, B, R', L', U', D', F', B', R2, L2, U2, D2, F2, B2`. The node does not validate syntax; it forwards moves as-is.
- Output: For each input move, publishes the same token on `cube_move` immediately after writing to serial.
- Ack requirement: For each move, the ESP32 must eventually send a line `DONE` to allow progression.

## Configuration Notes

- Serial port path: hard-coded to `/dev/ttyUSB0`. If your ESP32 appears elsewhere (e.g., `/dev/ttyACM0`), adjust the code or create a persistent udev rule mapping your board to `/dev/ttyUSB0`.
- Baud rate: 115200. Ensure firmware matches this rate.
- Ack token: `DONE`. Ensure firmware sends exactly this line (case-sensitive, newline-terminated) after each move completes.
- Ack timeout: 10 seconds. Adjust in code if your mechanics are slower.

## Logging

- Connection success: `INFO` (with ✅)
- Connection failure: `ERROR` (with ❌)
- Received solution and parsed move count: `INFO`
- Sent move: `INFO` (with 📤)
- ESP32 log lines: `INFO`
- Ack success: `INFO` (with ✅)
- Ack timeout: `ERROR` (with ❌)
- Missing serial connection when sending: `WARN` (with ⚠️)
- Sequence halted due to missing ACK: `WARN` (with ⚠️)

These symbols aid quick scanning of logs but are optional when scraping logs programmatically.

## Launch and Usage

Build and source the workspace using your ROS 2 toolchain. Then run the node via its console entry point.

Example workflow (assuming the workspace is already built):

- Run the node:
  - `ros2 run cube_solver cube_solver_node`

- Publish a test solution (replace with your actual solution):
  - `ros2 topic pub --once /cube_solution std_msgs/String '{data: "R U R' U' F2"}'`

- Monitor outgoing moves:
  - `ros2 topic echo /cube_move`

Ensure the ESP32 is connected and the firmware is listening on the same serial settings.

## Extensibility Considerations

If you need to extend or adapt the node:
- Parameterize serial settings and ack token using ROS parameters.
- Add input validation for moves to catch malformed tokens early.
- Support a queue or cancellation service to abort a running sequence.
- Provide a service/action interface for better lifecycle management and status reporting.
- Add a retry policy per move before aborting.
- Emit richer telemetry (e.g., timing per move, retries, error codes).

## Source Reference

- File: `src/cube_solver/cube_solver/cube_solver_node.py`
- Entry point: defined in `src/cube_solver/setup.py` under `console_scripts` as `cube_solver_node = cube_solver.cube_solver_node:main`.

## Minimal API Summary

- Class: `CubeSolverNode(Node)`
  - Subscriptions: `cube_solution` (`std_msgs/String`)
  - Publishers: `cube_move` (`std_msgs/String`)
  - Methods:
    - `solution_callback(msg: String) -> None`: Processes an incoming solution string and orchestrates move execution.
    - `send_move_with_ack(move: str) -> bool`: Sends a move to ESP32 and waits up to 10 seconds for an `DONE` acknowledgment.
- Function: `main(args=None) -> None`: Initializes ROS, spins the node, and handles graceful shutdown including closing the serial port.

## Testing Notes

For offline testing without hardware:
- Consider mocking the serial interface (e.g., with `pyserial-asyncio` or a stub that simulates `in_waiting` and `readline()` returning `DONE`).
- You can still observe `cube_move` publications in ROS to verify parsing and sequencing.

---

This document is intended to be a reliable operational reference for developers and operators interacting with the `cube_solver_node`. Update it if the serial protocol, topics, or behavior change.
