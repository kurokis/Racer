cd "$(dirname "$0")"
docker build -t interactive_labeling .
docker run -it --rm -p 8888:8888 --mount type=bind,source=$(pwd),target=/app interactive_labeling
