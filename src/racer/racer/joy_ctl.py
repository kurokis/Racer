import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.pub = self.create_publisher(Int8MultiArray, 'ts_joy', 10)
        self.sub = self.create_subscription(
            Int8MultiArray,
            'stick',
            self.listener_callback,
            10)
        rclpy.logging._root_logger.info('joystick_controller')
    
    def listener_callback(self, msg):
        data = msg.data
        throttle = data[1]
        steer = data[2]
        ts_msg = Int8MultiArray(data=[throttle, steer])
        self.pub.publish(ts_msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_controller = JoystickController()
    rclpy.spin(joystick_controller)
    
if __name__ == '__main__':
    main()
