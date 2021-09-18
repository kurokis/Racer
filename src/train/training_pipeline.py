import subprocess
import os
import shutil

if __name__=="__main__":
    # params
    parent_dir = os.path.dirname(os.path.abspath(__file__))

    # run bag2labels.py
    print("Running bag2labels/bag2labels.py")
    subprocess.run(["python3",parent_dir+"/bag2labels/bag2labels.py"])

    # clean up train/input_data
    print("Deleting folder: {}".format(parent_dir+"/train/input_data"))
    shutil.rmtree(parent_dir+"/train/input_data")

    # copy files from bag2labels/output_data to train/input_data
    print("Copying files form {} to {}".format(parent_dir+"/bag2labels/output_data", parent_dir+"/train/input_data"))
    shutil.copytree(parent_dir+"/bag2labels/output_data", parent_dir+"/train/input_data")

    # run train.py
    print("Running train/train.py")
    subprocess.run(["python3",parent_dir+"/train/train.py"])