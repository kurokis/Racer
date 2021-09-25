
# このファイルの場所にカレントディレクトリを移動
cd "$(dirname "$0")"

sudo docker build -f Dockerfile_all -t hgpt2/ubuntu_all .

