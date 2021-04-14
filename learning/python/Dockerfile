FROM python:3.8.9-buster

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /gpr

COPY ./source ./
RUN pip install ./interface

ENTRYPOINT ["bash"]
