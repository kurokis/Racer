# TensorRT 7.2でないと動かないので21.06をインストールする
FROM nvcr.io/nvidia/tensorrt:21.06-py3

RUN python3 -m pip install torch torchvision torchaudio torchsummary
RUN git clone https://github.com/NVIDIA-AI-IOT/torch2trt && cd torch2trt && python setup.py install

# このdockerfileでビルドしたイメージでは、/workspaces以下でしかtorch2trtを認識しない
# そのためtorch2trtを必要とするスクリプトを動かすときはcd /workspacesを実行すること

WORKDIR /app

