#!/bin/sh

docker build -t notebook .
docker run --rm -it -p 8888:8888 -v "$PWD":/home/jovyan/work notebook
