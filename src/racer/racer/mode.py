import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray

class Mode(Node):
    def __init__(self):
        super().__init__('mode')
        self.pub = self.create_publisher(Int8, 'mode', 10)
        self.sub = self.create_subscription(
            String,
            'key',
            self.key_callback,
            10)
        self.sub = self.create_subscription(
            Int8MultiArray,
            'stick',
            self.stick_callback,
            10)
        rclpy.logging._root_logger.info('mode')

        self.MODE_MANUAL = 0
        self.MODE_AUTO = 1

        # initialize mode with manual
        self.mode = self.MODE_MANUAL
    
    def key_callback(self, msg):
        key = msg.data
        if key=="n":
            self.mode = self.MODE_AUTO
        elif key=="m":
            self.mode = self.MODE_MANUAL
        self.publish_mode()

    def stick_callback(self, msg):
        data = msg.data
        bottom_button = data[4]
        left_button = data[6]

        if left_button==1:
            self.mode = self.MODE_MANUAL
        elif bottom_button==1:
            self.mode = self.MODE_AUTO
        self.publish_mode()

    def publish_mode(self):
        mode_msg = Int8(data=self.mode)
        self.pub.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    mode = Mode()
    rclpy.spin(mode)
    
if __name__ == '__main__':
    main()
