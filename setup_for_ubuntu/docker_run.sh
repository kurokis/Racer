# このファイルの親フォルダにカレントディレクトリを移動
cd "$(dirname "$0")"
cd ..

#Docker実行:
# - 終了時にイメージ削除(--rm)
# - インタラクティブセッションで実行(-it)
# - リポジトリのルートを/appにbind mount(--mount)
# - 共有メモリサイズ512MB(必要に応じ調整)(--shm-size)
# - ホストPCにGUIを表示させる(--net, -e, -v, 事前のxhost +実行)
# - イメージ名ubunturos(docker_build.shで指定済み)
xhost +
sudo docker run --rm -it --mount type=bind,source=$(pwd),target=/app --shm-size=512m --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubunturos
