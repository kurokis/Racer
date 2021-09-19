# training_pipline.pyはros2 galacticやnumpy等のライブラリに依存するため、専用のdocker image上で実行する。

# カレントディレクトリをプロジェクトのルートに移動
cd "$(dirname "$0")"
cd ../..

#Docker実行:
# - 実行するコンテナの名前を指定(--name)
# - 終了時にイメージ削除(--rm)
# - インタラクティブセッションで実行(-it)
# - リポジトリのルートを/appにbind mount(--mount)
# - 共有メモリサイズ512MB(必要に応じ調整)(--shm-size)
# - ホストPCにGUIを表示させる(--net, -e, -v, 事前のxhost +実行)
# - ジョイスティック対応(--privileged, --device)
# - イメージ名ubunturos(docker_build.shで指定済み)
xhost +
sudo docker run --gpus all --name racer_train --rm --privileged \
    --mount type=bind,source=$(pwd),target=/app \
    --shm-size=512m \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/input/js0:/dev/input/js0 \
    --device=/dev/snd:/dev/snd \
    hgpt2/ubuntu_galactic python3 src/train/training_pipeline.py