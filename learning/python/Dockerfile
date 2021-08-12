FROM python:3.8.9-buster

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /gpr

COPY ./source ./source
RUN pip install source/gpr source/control

COPY ./scripts ./scripts
WORKDIR /gpr/scripts

ENTRYPOINT ["bash"]
