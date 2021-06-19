import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
from sensor_msgs.msg import Image

class NeuralController(Node):
    def __init__(self):
        super().__init__('neural_controller')
        self.pub = self.create_publisher(Int8MultiArray, 'throttle_steer', 10)
        self.sub1 = self.create_subscription(
            String,
            'key',
            self.listener_callback_1,
            10)
        self.sub2 = self.create_subscription(
            Image,
            'cam/camera/image_raw',
            self.listener_callback_2,
            10)
        self.get_logger().info("NeuralController initialized")
            
        # variable for self state
        self.control_active = True

        # variables for listener
        self.new_key = None
        self.new_key_available = False
        self.image_msg = None
        
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.throttle = 0.0
        self.steer = 0.0
        
    def listener_callback_1(self, msg):
        self.new_key = msg.data
        self.new_key_available = True

    def listener_callback_2(self, msg):
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
        self.image_msg = msg
        
    def read_new_key(self):
        if self.new_key_available:
            self.new_key_available = False    
            return self.new_key
        else:
            return None
    
    def timer_callback(self):
        key = self.read_new_key()
        if key is not None:
            if key=="n":
                # "n" for neural control
                self.control_active = True
                self.get_logger().info("Activating neural control")
            elif key=="m":
                # next to n for deactivating neural control
                self.control_active = False
                self.get_logger().info("Deactivating neural control")
        
        if self.control_active:
            data = [int(self.throttle), int(self.steer)]
            msg = Int8MultiArray(data=data)
            self.pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    
    neural_controller = NeuralController()
    
    rclpy.spin(neural_controller)
    
if __name__ == '__main__':
    main()

