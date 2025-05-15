import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/rosbot/camera_depth/image',  # Change this to your actual topic name
            self.image_callback,
            10)
        
        self.get_logger().info('Image subscriber node started.')

    def image_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image using OpenCV
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Allows OpenCV to update the image window
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image subscriber node.")
    
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
