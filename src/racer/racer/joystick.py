import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import termios
import tty
import pygame


def createMsg():
    throttle=int(joy.get_axis(1))
    steer=int(joy.get_axis(3))
    stop=int((joy.get_axis(2)+1)/2)
    pygame.event.pump()

    msg = Float32MultiArray()
    msg.data.aapend(throttle)
    msg.data.append(steer)
    msg.data.append(stop)
    return msg

    
def main(args=None):
    COEFF=100
    JOYSTICKNUMBER=0
    pygame.init()
    joy = pygame.joystick.Joystick(JOYSTICKNUMBER) 
    joy.init()

    rclpy.init(args=args)

    node = rclpy.create_node('joy_input')
    pub = node.create_publisher(String, 'joy', 10)
    
    try:
        while True:
            # axis1 = left stick(up down), axis3= right stick(left,right)
            # axis2 = LT trigger. Each axis range : -1 <= x <=1.
            msg = createMsg()
            pub.publish(msg)
            node.get_logger().info('Publishing: "%s"' % msg.data)
             
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()

