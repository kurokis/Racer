import torch
from torch2trt import torch2trt
from torchvision.models.alexnet import alexnet
from torch2trt import TRTModule

# reload the trt model from file
model_trt_reload = TRTModule()
model_trt_reload.load_state_dict(torch.load('alexnet_trt.pth'))
print("Reload complete")
