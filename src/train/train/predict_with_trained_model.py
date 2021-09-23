import multiprocessing
from pathlib import Path
import os
import torch
from torch import nn
import torch.optim as optim
import torchvision
from torchsummary import summary
from lib.dataset import MyDataset
import glob
from PIL import Image as PILImage
from torchvision import transforms
import numpy as np


def train_model(model, n_epochs, device):
    criterion = nn.MSELoss()
    #optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    parent_dir = os.path.dirname(os.path.abspath(__file__))
    dataset = MyDataset(parent_dir+"/input_data",parent_dir+"/input_data/labels.csv")
    n_samples = len(dataset) 
    train_size = int(len(dataset) * 0.8)
    val_size = n_samples - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    print("Size of training dataset:",len(train_dataset))
    print("Size of validation dataset:",len(val_dataset))
    n_workers = multiprocessing.cpu_count()
    trainloader = torch.utils.data.DataLoader(train_dataset, batch_size = 8, shuffle = True, num_workers = n_workers)

    for epoch in range(n_epochs): # loop over the dataset multiple times
        running_loss = 0.0
        for i, data in enumerate(trainloader, 0):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data
            inputs, labels = inputs.to(device), labels.to(device)

            # zero the parameter gradients
            optimizer.zero_grad()

            # forward + backward + optimize
            outputs = model(inputs)
            #print(outputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()

            print_interval = 100 # print on every 100 mini-batches
            print_interval = min(print_interval, len(trainloader))
            if i % print_interval == (print_interval-1):
                print('epoch: %d mini-batch: %5d loss: %.3f' %
                    (epoch + 1, i + 1, running_loss / print_interval))
                running_loss = 0.0

    print('Training finished')
    return model

if __name__=="__main__":
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print('Using {} device'.format(device))

    output_dim = 2
    model = torchvision.models.resnet18(pretrained=True)
    model.fc = torch.nn.Linear(512, output_dim)
    model = model.to(device)

    #model_path = "output_data/model.pt"
    #model.load_state_dict(torch.load(model_path, map_location=torch.device(device)))
    model.eval()

    image_files = glob.glob("input_data/*.jpg")
    image_pil = PILImage.open(image_files[0])
    transform = transforms.Compose([
            transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
            transforms.Resize((224, 224)), # for resnet
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
    image_tensor = transform(image_pil) # 3d tensor
    image_tensor = image_tensor.unsqueeze(0) # 画像が1枚しかないので、次元を一つ加えて4次元テンソルにする

    # 0番目の画像をjpg出力
    x = image_tensor
    ch1 = x[0][0].to('cpu').detach().numpy().copy()
    ch2 = x[0][1].to('cpu').detach().numpy().copy()
    ch3 = x[0][2].to('cpu').detach().numpy().copy()
    tmp = (ch1+ch2+ch3)/3

    #tmp = image_tensor[0][0].to('cpu').detach().numpy().copy()        
    img_pil = PILImage.fromarray((((ch1*0.225)+0.5)*255).astype(np.uint8))
    #img_pil = Image.fromarray((tmp*255).astype(np.uint8))
    img_pil.save('prediction_image.jpg')

    y_raw = model(image_tensor.to(device))
    y = y_raw.detach().cpu().numpy()
    print("model output: {}".format(y))
    

