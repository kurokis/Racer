import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
from sensor_msgs.msg import Image
import numpy as np
import torch
from torch import nn

class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.W = 160
        self.H = 120
        self.C = 3
        self.flatten = nn.Flatten()
        self.conv_relu_stack = nn.Sequential(
            nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=24, out_channels=32, kernel_size=5, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=5, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(1280, 30),
            nn.ReLU(),
            nn.Linear(30, 2),
        )

    def forward(self, x):
        # input: tensor of dimensions (batch_size, channels, width, height)
        # output: tensor of dimensions (batch_size, 2)
        y_pred = self.conv_relu_stack(x)
        return y_pred

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

        # neural network
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = NeuralNetwork().to(device)
        
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
            self.predict_neural()
            data = [int(100*self.throttle), int(100*self.steer)]
            msg = Int8MultiArray(data=data)
            self.pub.publish(msg)
    
    def predict_neural(self):
        if self.image_msg is None:
            return
        # convert image to numpy array to torch tensor
        msg = self.image_msg
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        x = torch.from_numpy(im.astype(np.float32)).clone()

        # (h, w, c) to (c, w, h)
        x = x.permute(2, 1, 0)

        # add batch dimension (h, w, c) to (1, h, w, c)
        x = x.unsqueeze(0)

        #self.get_logger().info(str(x.size()))

        # forward propagation
        y = self.model(x)

        # remove batch dimension
        y = y.squeeze()
        y_numpy = y.to('cpu').detach().numpy().copy()

        self.throttle = y_numpy[0]
        self.steer = y_numpy[1]
        self.get_logger().info(str(y_numpy))
        

def main(args=None):
    rclpy.init(args=args)
    neural_controller = NeuralController()
    
    rclpy.spin(neural_controller)
    
if __name__ == '__main__':
    main()

