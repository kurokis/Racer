@echo off

rem このファイルの親フォルダにカレントディレクトリを移動
cd /d %~dp0
cd ..

rem カレントディレクトリをREPO_ROOTに格納
set REPO_ROOT=%cd%

rem バックスラッシュをスラッシュに、ドライブ名(C or D)を小文字に変更、スペースを削除
set REPO_ROOT_LINUX_STYLE=%REPO_ROOT:\=/% 
set REPO_ROOT_LINUX_STYLE=%REPO_ROOT_LINUX_STYLE:C:=c:%
set REPO_ROOT_LINUX_STYLE=%REPO_ROOT_LINUX_STYLE:D:=d:%
set REPO_ROOT_LINUX_STYLE=%REPO_ROOT_LINUX_STYLE: =%
echo mount source: %REPO_ROOT_LINUX_STYLE%

rem Docker実行: 終了時にイメージ削除,ホストPCのポート6080をコンテナのポート80に接続,リポジトリのルートを/appにbind mount,共有メモリサイズ512MB(必要に応じ調整),イメージ名winros(docker_build.batで指定済み)
docker run --name racer --rm -p 6080:80 --mount type=bind,source=%REPO_ROOT_LINUX_STYLE%,target=/app --shm-size=512m winros
