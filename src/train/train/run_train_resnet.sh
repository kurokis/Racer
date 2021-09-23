cd "$(dirname "$0")"

docker run --gpus all -it --rm --shm-size=2048m --mount type=bind,source=$(pwd),target=/app train_cuda python3 train_resnet.py
