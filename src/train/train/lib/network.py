import torch
from torch import nn
from torchsummary import summary

class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.W = 160
        self.H = 60
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
            nn.Linear(640, 30),
            nn.ReLU(),
            nn.Linear(30, 2),
        )

    def forward(self, x):
        # input: tensor of dimensions (batch_size, channels, width, height)
        # output: tensor of dimensions (batch_size, 2)
        y_pred = self.conv_relu_stack(x)
        return y_pred

# test code
if __name__=="__main__":
    # select device (cpu or cuda)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print('Using {} device'.format(device))

    # initialize the model
    model = NeuralNetwork().to(device)

    # print model info
    print("model info:")
    print(model)

    # model summary
    print("model summary:")
    batch_size = 32
    summary(model, (3,160,60), batch_size)

    # prediction example
    print("forward propagation example")
    X = torch.rand(1, 3, 160, 60, device=device)
    y = model(X)
    y_numpy = y.to('cpu').detach().numpy().copy()
    print("input shape:{}".format(X.shape))
    print("input:{}".format(X))
    print("output shape:{}".format(y_numpy.shape))
    print("output:{}".format(y_numpy))

    # save model to file (convert to cpu before save)
    save_path = "model.pt"
    torch.save(model.to("cpu").state_dict(), save_path) 

    # load model from file (convert from cpu to appropriate device)
    #model = NeuralNetwork().to(device)
    #model.load_state_dict(torch.load(save_path, map_location=torch.device(device)))
    #model.eval()