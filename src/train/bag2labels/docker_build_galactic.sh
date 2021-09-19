
# このファイルの場所にカレントディレクトリを移動
cd "$(dirname "$0")"

sudo docker build -f Dockerfile_for_ubuntu_galactic -t hgpt2/ubuntu_galactic .
