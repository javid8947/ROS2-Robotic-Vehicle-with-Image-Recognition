import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image as Im
from std_msgs.msg import String
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np

# Global instances
bridge = CvBridge()
model = None
class_names = []

def load_model_and_classes(): 
    global model, class_names 
    model = tf.keras.models.load_model("resource/keras_model.h5") 
    with open("resource/labels.txt", "r") as file: 
        class_names = [line.strip() for line in file.readlines()] # Strip whitespace and newline 
        print("Model loaded")
        print("Class names:", class_names) 

class DetectImage(Node):

    def __init__(self):
        super().__init__('rec_camera')
        self.subscription = self.create_subscription(Im, 'camera', self.camera_image_ai, 10)
        self.publisher = self.create_publisher(String, 'arduino_command', 10)

    def camera_image_ai(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg)

        # Resize and normalize the image
        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
        image = cv2.resize(cv_image, (224, 224))
        normalized_image_array = (image.astype(np.float32) / 127.5) - 1
        data[0] = normalized_image_array

        # Make a prediction
        prediction = model.predict(data)
        index = np.argmax(prediction)
        confidence_score = prediction[0][index]

        # Map class names to prediction values
        class_confidences = {class_name.strip(): prediction[0][i] for i, class_name in enumerate(class_names)}
        
        # Determine the action based on the prediction
        # Makes sure that the confidence in the choice is above 60% first
        if confidence_score > 0.6:
            action = None
            if class_confidences["3 TURN LEFT"] > 0.6:
                action = "left"
            elif class_confidences["2 TURN RIGHT"] > 0.6:
                action = "right"
            elif class_confidences["1 STOP"] > 0.6:
                action = "stop"

            if action:
                msg = String()
                msg.data = action
                self.publisher.publish(msg)
                # :.2% formats it to 2 decimal points, and gives as percentage
                self.get_logger().info(f'Publishing {action}, confidence {confidence_score:.2%}')
            else:
                self.get_logger().info(f'Nothing detected, confidence {confidence_score:.2%}')

def main(args=None):
    rclpy.init(args=args)
    load_model_and_classes()
    detect_image = DetectImage()
    rclpy.spin(detect_image)
    detect_image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
