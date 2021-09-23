cd "$(dirname "$0")"

docker run --gpus all -it --rm --mount type=bind,source=$(pwd),target=/app train_cuda bash
