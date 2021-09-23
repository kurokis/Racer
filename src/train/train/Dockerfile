FROM nvidia/cuda:11.4.2-cudnn8-devel-ubuntu20.04
#11.4.2-cudnn8-devel-ubuntu20.04 (11.4.2/ubuntu2004/devel/cudnn8/Dockerfile)

RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get -y install python3
RUN apt-get -y install python3-pip
RUN apt-get -y install nano wget curl

RUN pip3 install numpy pandas
RUN pip3 install torch torchvision torchsummary

WORKDIR /app

