cd "$(dirname "$0")"
cd ..

docker run --gpus all -it --rm --mount type=bind,source=$(pwd),target=/app trt bash tensorrt/run_convert_to_trt_in_container.sh
