# bag2labelsはros2 galacticに依存するため、専用のdocker image上で実行する。
# カレントディレクトリをプロジェクトのルートに移動
cd "$(dirname "$0")"
cd ..

sudo docker run --gpus all --rm \
    --mount type=bind,source=$(pwd),target=/app \
    hgpt2/ubuntu_galactic python3 bag2labels/bag2labels.py
