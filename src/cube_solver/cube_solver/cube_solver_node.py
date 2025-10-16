"""
cube_solver_node
=================

A ROS2 node that consumes a full Rubik's Cube solution string and streams
individual moves to an ESP32-based hardware controller over a serial link.

Responsibilities:
- Subscribe to a solution topic (std_msgs/String) carrying a space-separated
  sequence of cube moves (e.g., "R U R' U' F2").
- Send each move to the ESP32 on /dev/ttyUSB0 at 115200 baud, one per line.
- Wait for a per-move acknowledgment line "DONE" from the ESP32 before sending
  the next move; aborts on timeout or serial errors.
- Publish each move to a telemetry topic for UI/monitoring purposes.

Interfaces:
- Subscribes:  /cube_solution (std_msgs/msg/String)
- Publishes:   /cube_move    (std_msgs/msg/String)
- Serial port: /dev/ttyUSB0, 115200 baud, 1s read timeout; 10s total wait per move

Notes:
- The node logs connection status, incoming solutions, per-move progress,
  acknowledgments, and error conditions.
- If the serial device is unavailable at startup, the node continues to run,
  but attempts to send moves will warning-log and return failure.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class CubeSolverNode(Node):
    """
    ROS2 node that streams cube moves to an ESP32 with ACK handshaking.

    This node bridges high-level cube solutions to low-level actuator control
    by:
    - Listening for complete solution strings on the ``cube_solution`` topic.
    - Parsing the string into discrete moves.
    - Sending each move to an ESP32 over a serial connection.
    - Waiting for a ``DONE`` acknowledgment per move before proceeding.
    - Publishing each move on the ``cube_move`` topic for dashboards.

    Attributes
    ----------
    solution_sub : rclpy.subscription.Subscription
        Subscription to receive solution strings.
    move_pub : rclpy.publisher.Publisher
        Publisher emitting each move as it is sent to hardware.
    serial_port : serial.Serial | None
        Open serial connection to the ESP32, or ``None`` if unavailable.

    """
    def __init__(self):
        super().__init__('cube_solver_node')

        # ROS 2 interfaces: subscribe for solutions, publish each executed move.
        self.solution_sub = self.create_subscription(String, 'cube_solution', self.solution_callback, 10)
        self.move_pub = self.create_publisher(String, 'cube_move', 10)

        # Establish serial connection to the ESP32 controller.
        # Port and baud rate must match firmware settings.
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("✅ Connected to ESP32 on /dev/ttyUSB0")
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f"❌ Could not connect to ESP32: {e}")

        self.get_logger().info("Cube Solver Node started, waiting for solutions...")

    def solution_callback(self, msg):
        """
        Process an incoming cube solution string.

        The input is expected to be a space-separated list of moves. Each move
        is sent to the ESP32 with a trailing newline. The method waits for an
        acknowledgment (the literal line ``"DONE"``) for up to 10 seconds per
        move. If any move fails to receive an ACK, the sequence is aborted.

        Parameters
        ----------
        msg : std_msgs.msg.String
            The message containing the full cube solution, e.g. "R U R' U'".

        """
        solution = msg.data.strip()
        moves = solution.split()

        self.get_logger().info(f"Received solution: {solution}")
        self.get_logger().info(f"Parsed {len(moves)} moves.")

        for move in moves:
            # Send this move to the ESP32 and block until ACK or timeout.
            success = self.send_move_with_ack(move)
            if not success:
                self.get_logger().warn(f"⚠️ No ACK for move '{move}', stopping sequence.")
                break

        self.get_logger().info("✅ All moves executed (or stopped on error).")

    def send_move_with_ack(self, move):
        """
        Send one move to the ESP32 and wait for a ``DONE`` acknowledgment.

        The move is written to the serial port followed by a newline. The
        method then polls the serial buffer for up to 10 seconds awaiting a
        line equal to ``DONE``. Any other lines are logged at INFO level.

        Parameters
        ----------
        move : str
            A single cube move token (e.g., "R", "U'", "F2").

        Returns
        -------
        bool
            ``True`` if a ``DONE`` acknowledgment was received within the
            timeout; ``False`` if the serial port is unavailable, an error
            occurs, or the ACK times out.

        """
        if not self.serial_port:
            self.get_logger().warn("⚠️ ESP32 not connected.")
            return False

        try:
            # Send move
            self.serial_port.write((move + '\n').encode('utf-8'))
            self.get_logger().info(f"📤 Sent to ESP32: {move}")

            # Publish move (for dashboard)
            self.move_pub.publish(String(data=move))

            # Wait for acknowledgement
            start_time = time.time()
            while time.time() - start_time < 10:  # 10s timeout
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line == "DONE":
                        self.get_logger().info(f"✅ ACK received for move: {move}")
                        return True
                    else:
                        self.get_logger().info(f"ESP32 says: {line}")
                time.sleep(0.1)

            self.get_logger().error(f"❌ Timeout waiting for ACK for move: {move}")
            return False

        except Exception as e:
            self.get_logger().error(f"⚠️ Serial communication error: {e}")
            return False

def main(args=None):
    """
    Entry point: initialize ROS, run the node, and ensure clean shutdown.

    This function initializes the rclpy context, creates and spins the
    ``CubeSolverNode`` until interrupted, then closes the serial port (if open),
    destroys the node, and shuts down rclpy.
    
    """
    rclpy.init(args=args)
    node = CubeSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial port explicitly to release the device handle on exit.
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
