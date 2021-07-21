# このファイルの親フォルダにカレントディレクトリを移動
cd "$(dirname "$0")"
cd ..

# コンテナ内でbashを起動する
sudo docker exec -it racer bash
