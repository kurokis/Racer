import subprocess
import os
import shutil
import time
import datetime

if __name__=="__main__":
    # params
    parent_dir = os.path.dirname(os.path.abspath(__file__))

    # clean up bag2labels/output_data
    if os.path.exists(parent_dir+"/bag2labels/output_data"):
        print("Deleting folder: {}".format(parent_dir+"/bag2labels/output_data"))
        shutil.rmtree(parent_dir+"/bag2labels/output_data")

    # run bag2labels.py
    print("Running bag2labels/bag2labels.py")
    result = subprocess.run(["bash",parent_dir+"/bag2labels/run_bag2labels.sh"], stdout=subprocess.PIPE)
    print("Running bag2labels.py complete at {}".format(datetime.datetime.now()))

    # clean up train/input_data
    if os.path.exists(parent_dir+"/train/input_data"):
        print("Deleting folder: {}".format(parent_dir+"/train/input_data"))
        shutil.rmtree(parent_dir+"/train/input_data")

    # copy files from bag2labels/output_data to train/input_data
    print("Copying files from {} to {}".format(parent_dir+"/bag2labels/output_data", parent_dir+"/train/input_data"))
    shutil.copytree(parent_dir+"/bag2labels/output_data", parent_dir+"/train/input_data")
    print("Copying files complete at {}".format(datetime.datetime.now()))

    # run train.py
    if result.returncode==0:
        print("Running train/train.py {}".format(datetime.datetime.now()))
        subprocess.run(["bash",parent_dir+"/train/run_train_resnet.sh"])
    else:
        print("bag2labels.py failed")

    # copy files from train/output_data to train/input_data
    print("Copying files from {} to {}".format(parent_dir+"/train/output_data", parent_dir+"/tensorrt/input_data"))
    shutil.copytree(parent_dir+"/train/output_data", parent_dir+"/tensorrt/input_data")
    print("Copying files complete at {}".format(datetime.datetime.now()))

    # run conver_to_trt.py
    if result.returncode==0:
        print("Running tensorrt/convert_to_trt.py {}".format(datetime.datetime.now()))
        subprocess.run(["bash",parent_dir+"/tensorrt/docker_run.sh"])
    else:
        print("convert_to_trt.py failed")
