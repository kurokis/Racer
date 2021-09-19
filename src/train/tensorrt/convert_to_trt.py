import os
import sys
import torch
from torch2trt import torch2trt
from torchvision.models.alexnet import alexnet
from torch2trt import TRTModule

sys.path.append(os.path.join(os.path.dirname(__file__), '../train/lib'))

from network import NeuralNetwork

#model = alexnet(pretrained=True).eval().to('cuda')

if __name__=="__main__":
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = NeuralNetwork().to(device)
    model_path = os.path.join(os.path.dirname(__file__), "input_data/model.pt")
    if os.path.exists(model_path):
        print("Neural network model exists at {}. Loading weights from the model.".format(model_path))
        model.load_state_dict(torch.load(model_path, map_location=torch.device(device)))
        model.eval()
    else:
        print("Neural network model does not exist. Using initial values for model weights.")
    
    model = model.eval().to(device)

    x = torch.ones((1,3,160,60)).to(device) # batch_size, channel, width, height

    # convert alexnet to TensorRT
    print("Converting model to TensorRT")
    model_trt = torch2trt(model, [x])

    y = model(x)
    y_trt = model_trt(x)
    print("Comparison with the original model")
    print(torch.max(torch.abs(y-y_trt)))

    # save trt alexnet
    output_path = os.path.join(os.path.dirname(__file__), "output_data/model_trt.pt")
    torch.save(model_trt.state_dict(), output_path)

