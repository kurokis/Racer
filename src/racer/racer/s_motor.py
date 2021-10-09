import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist

class SimMotor(Node):
    def __init__(self):
        super().__init__('s_motor')
        self.sub = self.create_subscription(
            Int8MultiArray,
            'throttle_steer',
            self.listener_callback,
            10)
        self.pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)

    def listener_callback(self, msg):
        data = msg.data
        throttle = data[0]
        steer = data[1]

        throttle *= 1.0
        steer *= 1.0
        
        msg = Twist()
        # linear.x: target speed in m/s
        # angular.z: target steering angle in rad
        msg.linear.x = (throttle/100.0) * 2.0

        # when the throttle is in reverse, angular command must be inverted
        # to steer in the correct direction
        steer_direction = 1.0
        if throttle<0:
            steer_direction = -1.0
        msg.angular.z = (steer/100.0) * (30/57.3) * steer_direction
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    s_motor = SimMotor()    
    rclpy.spin(s_motor)
  
if __name__ == '__main__':
    main()

