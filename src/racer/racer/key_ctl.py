import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray

#class KeyListener(Node):
#    def __init__(self):
#        super().__init__('key_listener')
#        self.subscription = self.create_subscription(
#            String,
#            'key',
#            self.listener_callback,
#            10)
#        self.subscription
#        self.new_key = None
#        self.new_key_available = False
#
#    def listener_callback(self, msg):
#        self.new_key = msg.data
#        self.new_key_available = True
#        #self.get_logger().info('Received key: "%s"' % msg.data)
#    
#    def read(self):
#        if self.new_key_available:
#            self.new_key_available = False    
#            return self.new_key
#        else:
#            return None
#
#class CommandPublisher(Node):
#    def __init__(self):
#        super().__init__('command_publisher')
#        self.pub = self.create_publisher(Int8MultiArray, 'throttle_steer', 10)
#        
#    def publish(self, msg):
#        self.pub.publish(msg)
#        self.get_logger().info("Publishing:{}".format(msg.data))

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.pub = self.create_publisher(Int8MultiArray, 'throttle_steer', 10)
        self.sub = self.create_subscription(
            String,
            'key',
            self.listener_callback,
            10)
            
        # variables for listener
        self.new_key = None
        self.new_key_available = False
            
        
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.throttle = 0.0
        self.steer = 0.0
        
    def listener_callback(self, msg):
        self.new_key = msg.data
        self.new_key_available = True
        
    def read_new_key(self):
        if self.new_key_available:
            self.new_key_available = False    
            return self.new_key
        else:
            return None
        
    def timer_callback(self):
        key = self.read_new_key()
        if key is not None:
            # keys:
            # q w e
            # a s d
            # z x c
            #
            # w: positive throttle
            # a: steer left
            # q: positive throttle + steer left
            # s: stop
            # etc.
            
            MAX_ABS_THROTTLE = 100
            MAX_ABS_STEER = 100
            DELTA_THROTTLE = 10
            DELTA_STEER = 10
            
            # update throttle
            if (key=="w" or key=="q" or key=="e"): # forward
                self.throttle += DELTA_THROTTLE
            elif (key=="x" or key=="z" or key=="c"): # backward
            	self.throttle -= DELTA_THROTTLE
            
            # update steer
            if (key=="a" or key=="q" or key=="z"): # left
                self.steer += DELTA_STEER
            elif (key=="d" or key=="e" or key=="c"): # right
                self.steer -= DELTA_STEER
            
            # other key input
            if key=="s":
            	self.throttle = 0.0
            	self.steer = 0.0
                
            # saturate throttle and steer
            self.throttle = max(min(self.throttle, MAX_ABS_THROTTLE),-MAX_ABS_THROTTLE)
            self.steer = max(min(self.steer, MAX_ABS_STEER),-MAX_ABS_STEER)
        else:
            self.throttle *= 0.8
            self.steer *= 0.8
        
        data = [int(self.throttle), int(self.steer)]
        msg = Int8MultiArray(data=data)
        self.pub.publish(msg)
        #self.get_logger().info("Publishing:{}".format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_controller = KeyboardController()
    
    #while rclpy.ok():
    #    rclpy.spin_once(keyboard_controller.sub, timeout_sec=0)
    #    rclpy.spin_once(keyboard_controller, timeout_sec=0)
    rclpy.spin(keyboard_controller)
    
if __name__ == '__main__':
    main()

