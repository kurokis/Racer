import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray

class Priority(Node):
    def __init__(self):
        super().__init__('mode')
        self.pub = self.create_publisher(Int8MultiArray, 'throttle_steer', 10)
        self.sub1 = self.create_subscription(
            Int8,
            'mode',
            self.mode_callback,
            10)
        self.sub2 = self.create_subscription(
            Int8MultiArray,
            'ts_key',
            self.key_ctl_callback,
            10)
        self.sub3 = self.create_subscription(
            Int8MultiArray,
            'ts_joy',
            self.joy_ctl_callback,
            10)
        self.sub4 = self.create_subscription(
            Int8MultiArray,
            'ts_nn',
            self.nn_ctl_callback,
            10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        rclpy.logging._root_logger.info('mode')

        self.MODE_MANUAL = 0
        self.MODE_AUTO = 1

        # initialize mode with manual
        self.mode = self.MODE_MANUAL
        self.ts_key = [0, 0]
        self.ts_joy = [0, 0]
        self.ts_nn = [0, 0]
    
    def mode_callback(self, msg):
        self.mode = msg.data

    def key_ctl_callback(self, msg):
        data = msg.data
        self.ts_key = [data[0], data[1]]
    
    def joy_ctl_callback(self, msg):
        data = msg.data
        self.ts_joy = [data[0], data[1]]

    def nn_ctl_callback(self, msg):
        data = msg.data
        self.ts_nn = [data[0], data[1]]

    def timer_callback(self):
        ts = [0, 0] # throttle, steer
        if self.mode==self.MODE_AUTO:
            ts = self.ts_nn
        elif self.mode==self.MODE_MANUAL:
            # prioritize keyboard over joystick
            if (self.ts_key[0]!=0) or (self.ts_key[1]!=0):
                ts = self.ts_key
            else:
                ts = self.ts_joy
        ts_msg = Int8MultiArray(data=ts)
        self.pub.publish(ts_msg)

def main(args=None):
    rclpy.init(args=args)
    priority = Priority()
    rclpy.spin(priority)
    
if __name__ == '__main__':
    main()
