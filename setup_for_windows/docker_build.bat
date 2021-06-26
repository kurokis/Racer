@echo off

rem このファイルの親フォルダにカレントディレクトリを移動
cd /d %~dp0
cd ..

rem リポジトリのルートに元から存在するDockerfile(Jetson Nano用)を一時退避させる。Jetson Nano用をメインとし、Windows用のDockerfileは補助の位置づけ。
if exist Dockerfile (
  move Dockerfile Dockerfile_original
) else (
  echo Dockerfile does not exist
)

copy setup_for_windows\Dockerfile_for_windows Dockerfile

set REPOSITORY_ROOT=%cd%
echo %REPOSITORY_ROOT%

docker build -t winros .

rem 一時退避させたDockerfile(Jetson Nano用)を元に戻す
if exist Dockerfile_original (
  move Dockerfile_original Dockerfile
)

pause