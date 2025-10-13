import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class LidTaskPublisher(Node):
    def __init__(self):
        super().__init__('lid_task_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(String, '/lid_task', 10)

        # Wait a few seconds before publishing
        self.timer = self.create_timer(5.0, self.publish_once)
        self.has_published = False

    def publish_once(self):
        if self.has_published:
            return

        task_msg = String()
        # Edit this line with your desired task
        task_msg.data = 'red_lid:0,silver_lid:0,black_lid:1'
        self.publisher_.publish(task_msg)

        self.get_logger().info(f'Published task: {task_msg.data}')
        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = LidTaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

