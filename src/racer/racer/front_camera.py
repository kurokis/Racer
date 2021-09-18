import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class FrontCamera(Node):
    def __init__(self):
        super().__init__('front_camera')
        self.publisher_ = self.create_publisher(Image, 'front_camera_image', 10)
        #self.publisher_ = self.create_publisher(Image, 'cam/camera/image_raw', 10)
        timer_period = 0.03 # seconds
        self.i = 0
        self.im_list = []
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        isCaptured,img=self.cap.read()
        if not isCaptured:
            return False
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(img), "rgb8"))
        #self.get_logger().info('Publishing an image')
        return True

    def gstreamer_pipeline(self,
        capture_width=160, capture_height=60,
        display_width=160, display_height=60,
        framerate=30, exposure_time_min= 1,exposure_time_max=30, # ms
        flip_method=0):

        exposure_time_min = exposure_time_min * 1000000 #ms to ns
        exposure_time_max = exposure_time_max * 1000000 #ms to ns

        exp_time_str = '"' + str(exposure_time_min) + ' ' + str(exposure_time_max) + '"'

        return (
            'nvarguscamerasrc '
            'name="NanoCam" '
            'do-timestamp=true '
            'timeout=0 '                               # 0 - 2147483647
            'blocksize=-1 '                            # block size in bytes
            'num-buffers=-1 '                          # -1..2147483647 (-1=ulimited) num buf before sending EOS
            'sensor-mode=-1 '                          # -1..255, IX279 0(3264x2464,21fps),1 (3264x1848,28),2(1080p.30),3(720p,60),4(=720p,120)
            'tnr-strength=-1 '                         # -1..1
            'tnr-mode=1 '                              # 0,1,2
    #        'ee-mode=0'                               # 0,1,2
    #        'ee-strength=-1 '                         # -1..1
            'aeantibanding=1 '                         # 0..3, off,auto,50,60Hz
            'bufapi-version=false '                    # new buffer api
    #        'maxperf=true '                            # max performance
            'silent=true '                             # verbose output
            'saturation=1 '                            # 0..2
            'wbmode=1 '                                # white balance mode, 0..9 0=off 1=auto
            'awblock=false '                           # auto white balance lock
            'aelock=false '                             # auto exposure lock(=true)
            'exposurecompensation=0 '                  # -2..2
            'exposuretimerange=%s '                    # "13000 683709000"
            'gainrange="1.0 10.625" '                  # "1.0 10.625"
            'ispdigitalgainrange="1 8" '             # "1 8"
            #
            '! video/x-raw(memory:NVMM), '
            'width=(int)%d, '
            'height=(int)%d, '
            'format=(string)NV12, '
            'framerate=(fraction)%d/1 '
            #
            '! nvvidconv flip-method=%d '
            '! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx '
            #
            '! videoconvert '
            '! video/x-raw, format=(string)BGR '
            #
            '! appsink'
            % (
                exp_time_str,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )


def main(args=None):
    rclpy.init(args=args)

    front_camera = FrontCamera()

    rclpy.spin(front_camera)
    
if __name__ == '__main__':
    main()