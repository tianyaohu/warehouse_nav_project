import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementController(Node):

    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher = self.create_publisher(Twist, 'robot/cmd_vel', 10)

    def move_for_x_sec(self, linear_x, angular_z, duration_sec):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        start_time = time.time()
        while (time.time() - start_time) < duration_sec and rclpy.ok():
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Moving: Linear={linear_x}, Angular={angular_z}")
            time.sleep(0.1)  # Adjust the sleep duration as needed

        # Stop the robot after the specified duration
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        self.get_logger().info("Movement completed.")

def main(args=None):
    rclpy.init(args=args)

    move_robot_node = MovementController()

    try:
        move_robot_node.move_for_x_sec(linear_x=0.2, angular_z=0.4, duration_sec=5.0)
    except KeyboardInterrupt:
        pass

    move_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
