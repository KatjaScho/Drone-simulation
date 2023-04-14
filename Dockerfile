FROM gboeing/osmnx:latest

RUN mamba install --yes gcc unzip
RUN pip install keplergl