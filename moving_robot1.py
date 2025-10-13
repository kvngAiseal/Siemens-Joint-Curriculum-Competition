import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import cv2
from ultralytics import YOLO
import time

class UR10LidPicker(Node):
    def __init__(self, selected_tasks):
        super().__init__('ur10_lid_picker')

        self.selected_tasks = selected_tasks  # Dict: {'red_lid': 3, ...}
        self.model = YOLO("Yolo_4_lids.pt")
        self.cap = cv2.VideoCapture(0)
        self.detected_colors = set()

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.task_pub = self.create_publisher(String, '/lid_task', 10)

        # Timers
        self.initial_counter = 0
        self.final_counter = 0
        self.initial_timer = None
        self.final_timer = None

        # Threads
        self.server_thread = threading.Thread(target=self.run_socket_server, daemon=True)
        self.detection_thread = threading.Thread(target=self.detect_lid_color, daemon=True)
        self.server_thread.start()
        self.detection_thread.start()

        self.get_logger().info("Waiting for selected lid colors to be detected...")

    def detect_lid_color(self):
        while rclpy.ok() and len(self.detected_colors) < len(self.selected_tasks):
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame from camera.")
                continue

            results = self.model(frame, verbose=False)[0]
            names = self.model.names

            for cls_id in results.boxes.cls:
                label = names[int(cls_id)]

                # Check if it's a selected color and not already detected
                if label in self.selected_tasks and label not in self.detected_colors:
                    self.get_logger().info(f"Detected selected lid: {label}")
                    self.detected_colors.add(label)

                    # Publish to /lid_task with quantity
                    msg = String()
                    msg.data = f"{label}: {self.selected_tasks[label]}"
                    self.task_pub.publish(msg)
                    self.get_logger().info(f"Published to /lid_task: {msg.data}")

                    # Start initial status message once
                    if self.initial_timer is None:
                        self.initial_timer = self.create_timer(1.0, self.publish_initial_message)

            time.sleep(0.5)

    def publish_initial_message(self):
        if self.initial_counter < 20:
            msg = String()
            msg.data = "UR10_1 is ready to start."
            self.status_pub.publish(msg)
            self.get_logger().info(f"[{self.initial_counter+1}/20] Published: {msg.data}")
            self.initial_counter += 1
        else:
            self.initial_timer.cancel()

    def publish_final_message(self):
        if self.final_counter < 25:
            msg = String()
            msg.data = "UR10_1: Finished loading the package."
            self.status_pub.publish(msg)
            self.get_logger().info(f"[{self.final_counter+1}/25] Published: {msg.data}")
            self.final_counter += 1
        else:
            self.final_timer.cancel()

    def run_socket_server(self):
        host = '0.0.0.0'
        port = 5000

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((host, port))
            server.listen(5)

            while True:
                conn, addr = server.accept()
                with conn:
                    self.get_logger().info(f"UR10 connected from {addr}")
                    data = conn.recv(1024)
                    if data:
                        message = data.decode().strip()
                        self.get_logger().info(f"Received from UR10: {message}")

                        if message.lower() == "done" and self.detected_colors:
                            self.final_timer = self.create_timer(1.0, self.publish_final_message)

def parse_lid_input():
    print("Select lid tasks (e.g., red_lid=3,black_lid=2):")
    user_input = input().strip()
    selected_tasks = {}
    valid_labels = {"red_lid", "black_lid", "silver_lid"}

    for entry in user_input.split(","):
        if "=" in entry:
            label, qty = entry.strip().split("=")
            label = label.strip()
            if label in valid_labels and qty.isdigit():
                selected_tasks[label] = int(qty)

    return selected_tasks

def main(args=None):
    rclpy.init(args=args)

    selected_tasks = parse_lid_input()

    if not selected_tasks:
        print("No valid lid tasks provided. Exiting.")
        return

    print(f"Selected tasks: {selected_tasks}")
    node = UR10LidPicker(selected_tasks)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


