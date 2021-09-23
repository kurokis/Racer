import glob
from pathlib import Path
import pandas as pd
from PIL import Image
import torch
from torchvision import transforms

class MyDataset(torch.utils.data.Dataset):
    def __init__(self, img_dir, labels_path):
        img_paths_ = glob.glob(str(Path(img_dir)/"*.jpg"))
        
        column_names = ["filename","throttle","steer"]
        df = pd.read_csv(labels_path, names=column_names)

        img_paths = []
        labels = []
        for path in img_paths_:
            df_match = df[df["filename"]==str(Path(path).name)]
            if len(df_match)==0:
                continue
            throttle = df_match["throttle"].values[0]
            steer = df_match["steer"].values[0]
            label = [throttle, steer]
            img_paths.append(path)
            labels.append(label)
        self.img_paths = img_paths
        self.labels = labels

        # define transforms to be applied on the image
        transform = transforms.Compose([
            transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
            transforms.Resize((224, 224)), # for resnet
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        self.transform = transform

    def __len__(self):
        return len(self.img_paths)

    def __getitem__(self, index):
        path = self.img_paths[index]
        image_pil = Image.open(path)
        image = self.transform(image_pil)
        label = torch.FloatTensor(self.labels[index])
        return image, label

# test code
if __name__=="__main__":
    dataset = MyDataset("../input_data","../input_data/labels.csv")
    print(len(dataset))

    # get the first element from the dataset
    image, label = dataset[0]
    print("image size:", image.shape)
    print("image:", image)
    print("label:", label)