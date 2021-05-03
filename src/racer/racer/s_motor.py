import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist

class ThrottleListener(Node):
    def __init__(self):
        super().__init__('throttle_listener')
        self.sub = self.create_subscription(
            Int8MultiArray,
            'throttle_steer',
            self.listener_callback,
            10)
        self.pub = self.create_publisher(Twist, '/demo/cmd_demo', 10)
        #self.sub

    def listener_callback(self, msg):
        data = msg.data
        throttle = data[0]
        steer = data[1]
        
        msg = Twist()
        msg.linear.x = throttle/100.0
        msg.angular.z = steer/100.0
        
        self.pub.publish(msg)
        #self.get_logger().info('Received throttle: "%s"' % throttle)

def main(args=None):
    rclpy.init(args=args)
    throttle_listener = ThrottleListener()    
    rclpy.spin(throttle_listener)
  
if __name__ == '__main__':
    main()

