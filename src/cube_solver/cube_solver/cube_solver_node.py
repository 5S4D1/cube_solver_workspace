import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CubeSolverNode(Node):
    def __init__(self):
        super().__init__('cube_solver_node')
        self.subscription = self.create_subscription(
            String,
            'cube_solution',
            self.solution_callback,
            10
        )
        self.get_logger().info("Cube Solver Node started, waiting for solutions...")

    def solution_callback(self, msg):
        self.get_logger().info(f"Received solution: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CubeSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
