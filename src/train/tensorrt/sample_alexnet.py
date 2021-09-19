import torch
from torch2trt import torch2trt
from torchvision.models.alexnet import alexnet
from torch2trt import TRTModule

model = alexnet(pretrained=True).eval().to('cuda')

x = torch.ones((1,3,224,224)).cuda()

# convert alexnet to TensorRT
print("Converting model to TensorRT")
model_trt = torch2trt(model, [x])

y = model(x)
y_trt = model_trt(x)
print("Comparison with the original model")
print(torch.max(torch.abs(y-y_trt)))

# save trt alexnet
torch.save(model_trt.state_dict(), 'alexnet_trt.pth')

# reload the trt model from file
model_trt_reload = TRTModule()
model_trt_reload.load_state_dict(torch.load('alexnet_trt.pth'))
print("Reload complete")
