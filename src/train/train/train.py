import multiprocessing
from pathlib import Path
import os
import torch
from torch import nn
import torch.optim as optim
from torchsummary import summary
from lib.network import NeuralNetwork
from lib.dataset import MyDataset

def train_model(model, n_epochs):
    criterion = nn.MSELoss()
    optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)

    parent_dir = os.path.dirname(os.path.abspath(__file__))
    dataset = MyDataset(parent_dir+"/input_data",parent_dir+"/input_data/labels.csv")
    n_samples = len(dataset) 
    train_size = int(len(dataset) * 0.8)
    val_size = n_samples - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    n_workers = multiprocessing.cpu_count()
    trainloader = torch.utils.data.DataLoader(train_dataset, batch_size = 1, shuffle = True, num_workers = n_workers)

    for epoch in range(n_epochs): # loop over the dataset multiple times
        running_loss = 0.0
        for i, data in enumerate(trainloader, 0):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data

            # zero the parameter gradients
            optimizer.zero_grad()

            # forward + backward + optimize
            outputs = model(inputs)
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

    # train the model
    n_epochs = 10
    model = NeuralNetwork().to(device)
    model = train_model(model, n_epochs)

    # save the trained model
    parent_dir = os.path.dirname(os.path.abspath(__file__))
    Path(parent_dir+"/output_data").mkdir(exist_ok=True)
    save_path = parent_dir+"/output_data/model.pt"
    torch.save(model.to("cpu").state_dict(), save_path) 

