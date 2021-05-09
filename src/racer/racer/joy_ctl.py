import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
import pygame

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.pub = self.create_publisher(Int8MultiArray, 'throttle_steer', 10)
        
        # init joystick
        JOYSTICKNUMBER=0
        pygame.init()
        self.joy = pygame.joystick.Joystick(JOYSTICKNUMBER) 
        self.joy.init()
        rclpy.logging._root_logger.info('joystick')
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
        
    
    def read_stick(self):
        # each joystciks' range is [-1,1]
        COEFF=100

        self.throttle=int(self.joy.get_axis(1))*COEFF
        self.steer=int(self.joy.get_axis(3))*COEFF
        #stop=int((self.joy.get_axis(2)+1)/2)
        pygame.event.pump()

    

    def timer_callback(self):
        self.get_logger().info("call_back")

        if self.joy.get_init():
            self.read_stick()
            data = [int(self.throttle), int(self.steer)]
            msg = Int8MultiArray(data=data)
            self.pub.publish(msg)
            self.get_logger().info("Publishing:{}".format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    
    joystick_controller = JoystickController()
    
    #while rclpy.ok():
    #    rclpy.spin_once(keyboard_controller.sub, timeout_sec=0)
    #    rclpy.spin_once(keyboard_controller, timeout_sec=0)
    rclpy.spin(joystick_controller)
    
if __name__ == '__main__':
    main()

