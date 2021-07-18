# このファイルの場所にカレントディレクトリを移動
cd "$(dirname "$0")"

sudo docker build -f Dockerfile -t racer-image .