import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
from sensor_msgs.msg import Image
import numpy as np
import torch
from torch import nn
import torchvision
import torchvision.transforms.functional as F
from torchvision import transforms

from PIL import Image as PILImage

class NeuralController(Node):
    def __init__(self):
        super().__init__('neural_controller')
        self.pub = self.create_publisher(Int8MultiArray, 'ts_nn', 10)
        self.sub2 = self.create_subscription(
            Image,
            'cam/camera/image_raw',
            self.image_callback,
            10)
        self.get_logger().info("NeuralController initialized")
        
        # variables for listener
        self.image_msg = None
        
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.throttle = 0.0
        self.steer = 0.0

        # neural network
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.device = device
        
        output_dim = 2
        model = torchvision.models.resnet18(pretrained=False)
        model.fc = torch.nn.Linear(512, output_dim)
        model = model.to(device)

        pkg_dir = get_package_share_directory('racer')
        model_path = os.path.join(pkg_dir, "params/model.pt")
        if os.path.exists(model_path):
            self.get_logger().info("Neural network model exists at {}. Loading weights from the model.".format(model_path))
            model.load_state_dict(torch.load(model_path, map_location=torch.device(device)))
            model.eval()
        else:
            self.get_logger().info("Neural network model does not exist. Using initial values for model weights.")
        self.model = model.to(device)
        
    def image_callback(self, msg):
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
        self.image_msg = msg
    
    def timer_callback(self):
        self.predict_neural()
        data = [int(100*self.throttle), int(100*self.steer)]
        msg = Int8MultiArray(data=data)
        self.pub.publish(msg)
        self.get_logger().info("NN node output: {}".format(data))
    
    def predict_neural(self):
        if self.image_msg is None:
            return
        # convert image to numpy array to torch tensor
        msg = self.image_msg
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        x = torch.from_numpy(im.astype(np.float32)).clone().to(self.device) # tensor of shape (h, w, c) 

        # convert range to 0-1
        x = x/255

        # (h, w, c) to (c, h, w)
        x = x.permute(2, 0, 1)

        # add batch dimension (c, h, w) to (1, c, h, w)
        x = x.unsqueeze(0)

        transform = transforms.Compose([
            #transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
            transforms.Resize((224, 224)), # for resnet
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        x = transform(x)

        # for debug
        # normalize: output[channel] = (input[channel] - mean[channel]) / std[channel]
        # apply reverse operation to obtain image
        #ch1 = x[0][0].to('cpu').detach().numpy().copy()
        #ch2 = x[0][1].to('cpu').detach().numpy().copy()
        #ch3 = x[0][2].to('cpu').detach().numpy().copy()
        #tmp = (ch1+ch2+ch3)/3
        #img_pil = PILImage.fromarray((((tmp*0.225)+0.5)*255).astype(np.uint8))
        #img_pil.save('transformed.jpg')
        #self.get_logger().info(str(ch1)) # 平均値や最大値が学習時から大きく乖離していないことを確認すること
        #self.get_logger().info("type of x: {}".format(str(type(x))))
        #self.get_logger().info("shape of x: {}".format(str(list(x.size()))))

        # forward propagation
        y = self.model(x)

        # remove batch dimension
        y = y.squeeze()
        y_numpy = y.to('cpu').detach().numpy().copy()

        self.get_logger().info("Raw model output: {}".format(y_numpy))
        throttle = y_numpy[0]
        throttle = float(min(1, max(0, throttle)))
        steer = y_numpy[1]
        steer = float(min(1, max(-1, steer)))
        
        self.throttle = throttle
        self.steer = steer
        

def main(args=None):
    rclpy.init(args=args)
    neural_controller = NeuralController()
    rclpy.spin(neural_controller)
    
if __name__ == '__main__':
    main()

