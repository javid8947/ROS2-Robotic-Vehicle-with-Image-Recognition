import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoControl(Node):

    def __init__(self):
        super().__init__('arduino_control')
        self.subscription = self.create_subscription(String, 'arduino_command', self.send_command_to_arduino, 10)
        
        # Initialize the serial port
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Change the port accordingly
            self.get_logger().info('Serial port initialized')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to initialize serial port: {e}')
            rclpy.shutdown()
            exit()

    def send_command_to_arduino(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        
        try:
            if msg.data == "right":
                self.serial_port.write(b'0')  # Sending '0' to turn on right LED
                self.serial_port.write(b'q')  # Sending 'q' to turn on left motor
                self.get_logger().info("Sent 'right' command to Arduino")
            elif msg.data == "left":
                self.serial_port.write(b'1')  # Sending '1' to turn on left LED
                self.serial_port.write(b'a')  # Sending 'a' to turn on right motor
                self.get_logger().info("Sent 'left' command to Arduino")
            elif msg.data == "stop":
                self.serial_port.write(b't')  # Sending 't' to turn all motors off
                self.get_logger().info("Sent 'stop' command to Arduino")
            else:
                self.get_logger().warn("Invalid command received.")
        except Exception as e:
            self.get_logger().error(f'Error sending command to Arduino: {e}')

def main(args=None):
    rclpy.init(args=args)
    arduino_control = ArduinoControl()
    try:
        rclpy.spin(arduino_control)
    except KeyboardInterrupt:
        pass
    finally:
        arduino_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
