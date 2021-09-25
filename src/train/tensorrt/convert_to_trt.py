import os
import sys
import torch
from torch2trt import torch2trt, TRTModule
import torchvision
from torchvision.models.alexnet import alexnet

if __name__=="__main__":
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    
    output_dim = 2
    model = torchvision.models.resnet18(pretrained=False)
    model.fc = torch.nn.Linear(512, output_dim)
    model = model.to(device)

    model_path = os.path.join(os.path.dirname(__file__), "input_data/model.pt")
    if os.path.exists(model_path):
        print("Neural network model exists at {}. Loading weights from the model.".format(model_path))
        model.load_state_dict(torch.load(model_path, map_location=torch.device(device)))
        model.eval()
    else:
        print("Neural network model does not exist.")
        raise Error
    
    model = model.eval().to(device)

    x = torch.ones((1,3,224,224)).to(device) # batch_size, channel, width, height

    # convert alexnet to TensorRT
    print("Converting model to TensorRT")
    model_trt = torch2trt(model, [x], fp16_mode=True)

    y = model(x)
    y_trt = model_trt(x)
    print("Comparison with the original model")
    print(torch.max(torch.abs(y-y_trt)))

    # save trt alexnet
    os.makedirs(os.path.join(os.path.dirname(__file__), "output_data"), exist_ok=True)
    output_path = os.path.join(os.path.dirname(__file__), "output_data/model_trt.pt")
    torch.save(model_trt.state_dict(), output_path)

