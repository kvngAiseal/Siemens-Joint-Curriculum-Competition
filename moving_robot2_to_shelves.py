import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

class TaskToUR10Controller(Node):
    def __init__(self):
        super().__init__('task_to_ur10_controller')

        # Predefined poses
        self.poses = {
            'red_lid': "[1.8, -1.57, -0.86, -0.72, 1.28, 1.62]",
            'silver_lid': "[1.63, -1.38, -1.39, -0.36, 1.52, 1.62]",
            'black_lid': "[1.64, -1.31, -1.81, -0.05, 1.52, 1.62]",
        }

        # Internal state
        self.detection_active = False
        self.pending_tasks = {}

        # ROS 2 interfaces
        self.ur10_pub = self.create_publisher(String, '/ur10_2/urscript_interface/script_command', 10)
        self.task_sub = self.create_subscription(String, '/lid_task', self.task_callback, 10)

        # Start socket server in a thread
        self.server_thread = threading.Thread(target=self.run_socket_server, daemon=True)
        self.server_thread.start()
        self.get_logger().info("Socket server started on port 5000")

    def task_callback(self, msg: String):
        self.get_logger().info(f"Task message received: {msg.data}")
        self.pending_tasks = self.parse_task_message(msg.data)

        if self.detection_active:
            self.get_logger().info("Already received 'ready', starting execution now.")
            threading.Thread(target=self.execute_tasks, daemon=True).start()
        else:
            self.get_logger().info("Task stored. Waiting for 'ready' from FlexPendant.")

    def parse_task_message(self, message: str):
        tasks = {}
        for entry in message.strip().split(','):
            if ':' in entry:
                key, val = entry.split(':')
                try:
                    tasks[key.strip()] = int(val.strip())
                except ValueError:
                    self.get_logger().warn(f"Invalid value for {key}: {val}")
        return tasks

    def execute_tasks(self):
        self.get_logger().info("Starting task execution.")
        for lid_type, count in self.pending_tasks.items():
            if lid_type not in self.poses or count <= 0:
                continue

            pose = self.poses[lid_type]
            for i in range(count):
                self.get_logger().info(f"Moving to {lid_type} shelf. Iteration {i+1} of {count}")
                self.move_to_pose(pose)

        self.get_logger().info("Finished executing all lid tasks.")
        self.detection_active = False  # Reset so we wait for 'ready' next time

    def move_to_pose(self, pose_string: str):
        # Send direct movej command (no function wrapper)
        command = f"movej({pose_string}, a=1.2, v=0.25)"
        msg = String()
        msg.data = command
        self.ur10_pub.publish(msg)
        self.get_logger().info(f"Published move command: {command}")
        
        # Wait approx. for motion to complete (non-blocking spin)
        start_time = time.time()
        while time.time() - start_time < 4.5:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_socket_server(self):
        host = '0.0.0.0'
        port = 5000
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(5)

            while True:
                conn, addr = server.accept()
                self.get_logger().info(f"Connection from {addr}")
                threading.Thread(target=self.handle_flexpendant_connection, args=(conn,), daemon=True).start()

    def handle_flexpendant_connection(self, conn):
        with conn:
            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    message = data.decode().strip().lower()
                    self.get_logger().info(f"Received from FlexPendant: {message}")

                    if message == "ready":
                        self.detection_active = True
                        if self.pending_tasks:
                            self.get_logger().info("Received 'ready'. Launching task execution.")
                            threading.Thread(target=self.execute_tasks, daemon=True).start()
                    elif message == "finished":
                        self.get_logger().info("Received 'finished' signal from FlexPendant.")

            except Exception as e:
                self.get_logger().error(f"Socket error: {e}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TaskToUR10Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

