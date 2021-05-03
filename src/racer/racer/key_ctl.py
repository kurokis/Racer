import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

class KeyListener(Node):
    def __init__(self):
        super().__init__('key_listener')
        self.subscription = self.create_subscription(
            String,
            'key',
            self.listener_callback,
            10)
        self.subscription
        self.new_key = None
        self.new_key_available = False

    def listener_callback(self, msg):
        self.new_key = msg.data
        self.new_key_available = True
        #self.get_logger().info('Received key: "%s"' % msg.data)
    
    def read(self):
        if self.new_key_available:
            self.new_key_available = False    
            return self.new_key
        else:
            return None

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.pub = self.create_publisher(Int8, 'topic', 10)
        
    def publish(self, msg):
        self.pub.publish(msg)
        self.get_logger().info("Publishing:{}".format(msg.data))

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.pub = CommandPublisher()
        self.sub = KeyListener()
        
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.throttle = 0.0
        
    def timer_callback(self):
        key = self.sub.read()
        if key is not None:
            if key=="w":
                self.throttle += 10
                self.throttle = min(self.throttle, 100)
            elif key=="s":
            	self.throttle -= 10
            	self.throttle = max(self.throttle, -100)
        else:
            self.throttle *= 0.8
        
        msg = Int8()
        msg.data = int(self.throttle)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_controller = KeyboardController()
    
    while rclpy.ok():
        rclpy.spin_once(keyboard_controller.sub, timeout_sec=0)
        rclpy.spin_once(keyboard_controller, timeout_sec=0)

    
if __name__ == '__main__':
    main()

