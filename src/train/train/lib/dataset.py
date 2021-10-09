import glob
from pathlib import Path
import pandas as pd
from PIL import Image, ImageOps
import torch
from torchvision import transforms

class MyDataset(torch.utils.data.Dataset):
    def __init__(self, img_dir):
        label_paths = glob.glob(str(Path(img_dir)/"**/labels.csv"))
        dfs = []
        for label_path in label_paths:
            print("Reading: {}".format(label_path))
            column_names = ["filename","throttle","steer"]
            df = pd.read_csv(label_path, names=column_names)
            df["filename"] = df["filename"].apply(lambda x: str(Path(label_path).parent/x))

            dfs.append(df)
        df = pd.concat(dfs) 
        
        img_paths = []
        labels = []
        hflips = []
        for index, row in df.iterrows():
            throttle = row["throttle"]
            steer = row["steer"]
            label = [throttle, steer]
            img_paths.append(row["filename"])
            labels.append(label)
            hflips.append(0)
            print(throttle, steer, row["filename"])
            
            # Horizontal flipping
            enable_hflip = True
            if enable_hflip:
                label = [throttle, -1*steer]
                img_paths.append(row["filename"])
                labels.append(label)
                hflips.append(1)

        self.img_paths = img_paths
        self.labels = labels
        self.hflips = hflips

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
        hflip = self.hflips[index]
        image_pil = Image.open(path)
        if hflip==1:
            image_pil = ImageOps.mirror(image_pil)
        image = self.transform(image_pil)
        label = torch.FloatTensor(self.labels[index])
        return image, label

# test code
if __name__=="__main__":
    dataset = MyDataset("../input_data")
    print(len(dataset))

    # get the first element from the dataset
    image, label = dataset[0]
    print("image size:", image.shape)
    print("image:", image)
    print("label:", label)