FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  pipenv \
  wget \
  tar \
  libffi-dev \
  libssl-dev \
  sudo \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/scr
RUN wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz \
  && tar xzf Python-3.7.9.tgz \
  && cd Python-3.7.9 \
  && ./configure --enable-optimizations \
  && make altinstall

WORKDIR /tmp/learning

RUN pip3 install pipenv --upgrade
RUN pipenv run pip install --upgrade setuptools

COPY ./source ./source
COPY ./Pipfile ./
COPY ./Pipfile.lock ./

# install pipenv
RUN pipenv install

ENTRYPOINT ["/usr/local/bin/pipenv", "run"]
CMD ["test_interface"]
