import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):

    def __init__(self):
        super().__init__('get_camera')
        self.publisher = self.create_publisher(Image, 'camera', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.get_frame)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.keep_running = True  # Flag to control the main loop

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        cv2.imshow('frame', frame) # Show camera frame on screen

        if cv2.waitKey(1) & 0xFF == 27:  # Escape key ASCII code is 27
            self.keep_running = False

        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        try:
            self.publisher.publish(image_message)
            self.get_logger().info('Publishing image')
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        while rclpy.ok() and node.keep_running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
