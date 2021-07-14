import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist

import json
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685


class motor_contoroller_via_pca9685(Node): # Node
    def __init__(self):
        super().__init__('r_motor')
        self.__duty_param=self.__import_param("/home/ryo/src/src/racer/params/motors.json")
        print(self.__duty_param)
        self.__i2c_bus = busio.I2C(SCL, SDA)
        self.__pca = PCA9685(self.__i2c_bus)
        self.__pca.frequency = self.__duty_param["frequency"]
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

    def __import_param(self,filename):
        f=open(filename,'r')
        return json.load(f)

    def __listener_callback(self,msg):
        data = msg.data
        throttle = data[0]
        steer = data[1]

        self.__change_steer_pwm(steer)
        self.__change_throttle_pwm(throttle)

     

    def __change_steer_pwm(self,steer):
        duty=self.__steer2pwm(steer)
        self.__pca.channels[self.__duty_param["servo"]["ch"]].duty_cycle = int(duty)

    def __change_throttle_pwm(self,throttle):
        duty=self.__throttle2pwm(throttle)
        self.__pca.channels[self.__duty_param["dcmotor"]["ch"]].duty_cycle = int(duty)


    def __steer2pwm(self,steer):
        min=int(self.__duty_param["servo"]["left"])
        mid=int(self.__duty_param["servo"]["center"])
        max=int(self.__duty_param["servo"]["right"])

        pwm=0
        if steer>0:
            pwm=mid+steer/100*(max-mid)
        else:
            pwm=mid+steer/100*(mid-min)
        
        return pwm

    def __throttle2pwm(self,throttle):
        min=self.__duty_param["dcmotor"]["forward"]
        mid_min=self.__duty_param["dcmotor"]["neutral_min"]
        mid_max=self.__duty_param["dcmotor"]["neutral_max"]
        max=self.__duty_param["dcmotor"]["back"]

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