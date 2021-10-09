import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

import json
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

class motor_contoroller_via_pca9685(Node): # Node
    def __init__(self):
        super().__init__('r_motor')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('servo.ch', 1),
                ('servo.left', None),
                ('servo.center', None),
                ('servo.right', None),
                ('dcmotor.ch', 2),
                ('dcmotor.forward', None),
                ('dcmotor.neutral_min', None),
                ('dcmotor.neutral_max', None),
                ('dcmotor.back', None),
                ('frequency', 59)
            ])

        self.__i2c_bus = busio.I2C(SCL, SDA)
        self.__pca = PCA9685(self.__i2c_bus)
        self.__pca.frequency = self.get_parameter("frequency").value

        self.sub = self.create_subscription(
            Int8MultiArray,
            'throttle_steer',
            self.__listener_callback,
            10)
        self.__change_steer_pwm(0)
        self.__change_throttle_pwm(0)

    def __del__(self):
        self.__change_steer_pwm(0)
        self.__change_throttle_pwm(0)

    def __listener_callback(self,msg):
        data = msg.data
        throttle = data[0]
        steer = data[1]

        self.__change_steer_pwm(steer)
        self.__change_throttle_pwm(throttle)

     

    def __change_steer_pwm(self,steer):
        duty=self.__steer2pwm(steer)
        self.__pca.channels[ self.get_parameter("servo.ch").value].duty_cycle = int(duty)

    def __change_throttle_pwm(self,throttle):
        duty=self.__throttle2pwm(throttle)
        self.__pca.channels[self.get_parameter("dcmotor.ch").value].duty_cycle = int(duty)


    def __steer2pwm(self,steer):
        min=int(self.get_parameter("servo.left").value)
        mid=int(self.get_parameter("servo.center").value)
        max=int(self.get_parameter("servo.right").value)

        pwm=0
        if steer>0:
            pwm=mid+steer/100*(max-mid)
        else:
            pwm=mid+steer/100*(mid-min)
        
        return pwm

    def __throttle2pwm(self,throttle):
        min=self.get_parameter("dcmotor.forward").value
        mid_min=self.get_parameter("dcmotor.neutral_min").value
        mid_max=self.get_parameter("dcmotor.neutral_max").value
        max=self.get_parameter("dcmotor.back").value

        pwm=0
        if throttle>0:
            pwm=mid_min-throttle/100*(mid_min-min)
        else:
            pwm=mid_max-throttle/100*(max-mid_max)
        
        return pwm



def main(args=None):
    rclpy.init(args=args)

    motor_contor = motor_contoroller_via_pca9685()
        
    rclpy.spin(motor_contor)

    
if __name__ == '__main__':
    main()