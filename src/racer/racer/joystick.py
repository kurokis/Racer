import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
import pygame

class Joystick(Node):
    def __init__(self):
        super().__init__('joystick')
        self.pub = self.create_publisher(Int8MultiArray, 'stick', 10)
        
        # init joystick
        JOYSTICKNUMBER=0
        pygame.init()
        self.joy = pygame.joystick.Joystick(JOYSTICKNUMBER) 
        self.joy.init()
        rclpy.logging._root_logger.info('joystick')

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.throttle = 0.0
        self.steer = 0.0

    
    def read_stick(self):
        # sticks (each stick's original range is [-1,1])
        #   axis 0: left stick, horizontal, left positive after conversion
        #   axis 1: left stick, vertical, up positive after conversion => throttle
        #   axis 3: right stick, horizontal, left positive after conversion => steer
        #   axis 4: right stick, vertical, up positive after conversion
        # buttons
        #   button 0: right hand, bottom button
        #   button 1: right hand, right button
        #   button 2: right hand, left button
        #   button 3: right hand, top button
        
        COEFF=100
        lsh = int(self.joy.get_axis(0)*COEFF*-1)
        lsv = int(self.joy.get_axis(1)*COEFF*-1)
        rsh = int(self.joy.get_axis(3)*COEFF*-1)
        rsv = int(self.joy.get_axis(4)*COEFF*-1)

        bb = int(self.joy.get_button(0))
        br = int(self.joy.get_button(1))
        bl = int(self.joy.get_button(2))
        bt = int(self.joy.get_button(3))
        
        pygame.event.pump()

        data = [lsh, lsv, rsh, rsv, bb, br, bl, bt] 
        return data
    
    def timer_callback(self):
        if self.joy.get_init():
            data = self.read_stick()
            msg = Int8MultiArray(data=data)
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joystick = Joystick()
    rclpy.spin(joystick)

if __name__ == '__main__':
    main()

