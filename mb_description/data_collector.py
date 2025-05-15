import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import csv
from datetime import datetime

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/rosbot/camera_rgb/image_color', self.image_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.latest_cmd = None
        self.data_dir = 'dataset'
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.data_dir, 'labels.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['image_filename', 'linear_x', 'linear_y', 'angular_z'])

        self.frame_count = 0
        self.save_every_n = 10  # Change this to control frequency (e.g., 5 = save 1 out of every 5 images)


    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def image_callback(self, msg):
        if self.latest_cmd is None:
            return
        
        self.frame_count += 1
        if self.frame_count % self.save_every_n != 0:
            return  # Skip this frame 


        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')
        image_filename = f"{timestamp}.jpg"
        image_path = os.path.join(self.data_dir, image_filename)
        cv2.imwrite(image_path, cv_image)

        self.csv_writer.writerow([
            image_filename,
            self.latest_cmd.linear.x,
            self.latest_cmd.linear.y,
            self.latest_cmd.angular.z
        ])
        self.get_logger().info(f"Saved {image_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
