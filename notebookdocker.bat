@ECHO OFF

docker build -t notebook .
docker run --rm -it -p 8888:8888 -v "%cd%":/home/jovyan/work notebook
