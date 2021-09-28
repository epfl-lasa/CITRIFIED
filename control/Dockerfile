FROM ubuntu:20.04 AS core-build-dependencies
ENV DEBIAN_FRONTEND=noninteractive

# install core compilation and access dependencies for building the libraries
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    make \
    cmake \
    git \
    curl \
    wget \
    lsb-release \
    gnupg2 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


FROM core-build-dependencies as development-dependencies

RUN apt-get update && apt-get install -y \
    libgtest-dev \
    build-essential \
    ssh \
    libssl-dev \
    gdb \
    clang \
    rsync \
    tar \
    python \
    sudo \
    iputils-ping \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# install gtest
WORKDIR /tmp/gtest_build
RUN cmake /usr/src/gtest \
  && make \
  && cp lib/* /usr/local/lib || cp *.a /usr/local/lib

RUN rm -rf /tmp/*


FROM development-dependencies as project-dependencies

RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    libsodium-dev \
    libzmq3-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
# get cppzmq bindings
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
  && tar -xzf cppzmq-4.7.1.tar.gz \
  && cd cppzmq-4.7.1 \
  && mkdir build \
  && cd build \
  && cmake .. -DCPPZMQ_BUILD_TESTS=OFF \
  && make -j4 install

# install KDL
RUN git clone -b v1.5.0 --single-branch https://github.com/orocos/orocos_kinematics_dynamics.git \
  && cd orocos_kinematics_dynamics/orocos_kdl \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install

# install json library header
RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
  && mv json.hpp /usr/local/include

# install control libraries
RUN git clone -b v3.1.0 --single-branch https://github.com/epfl-lasa/control_libraries.git \
  && mkdir -p control_libraries/source/build \
  && cd control_libraries/source/build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_ROBOT_MODEL="OFF" .. \
  && make -j \
  && make install

# clean up installation artefacts
RUN rm -rf /tmp/*
RUN ldconfig


FROM project-dependencies as remote-development

RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

RUN useradd -m remote && yes | passwd remote && usermod -s /bin/bash remote
WORKDIR /home/remote

RUN ( \
    echo '#!/bin/bash'; \
    echo 'mkdir -p /home/remote/.ssh'; \
    echo 'echo "$1" > /home/remote/.ssh/authorized_keys'; \
    echo 'chmod -R 755 /home/remote/.ssh'; \
    echo 'chown -R remote:remote /home/remote/.ssh'; \
    echo '/usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_development'; \
  ) > /.ssh_entrypoint.sh && chmod 744 /.ssh_entrypoint.sh

ENTRYPOINT ["/.ssh_entrypoint.sh"]


# tag a new build stage that copies the project source files
FROM project-dependencies as project-sources

WORKDIR /usr/src/CITRIFIED/control
COPY executables executables
COPY include include
COPY source source
COPY CMakeLists.txt CMakeLists.txt


# tag a new build stage that compiles and runs the test targets
FROM project-sources as tester

WORKDIR /usr/src/CITRIFIED/control/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=True \
  && make
RUN CTEST_OUTPUT_ON_FAILURE=1 make test


# tag a new build stage that compiles the sources
FROM project-sources as runtime

RUN useradd -ms /bin/bash citrified

WORKDIR /usr/src/CITRIFIED/control/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
  && make \
  && mkdir -p /home/citrified/executables \
  && find executables -maxdepth 1 -type f -exec test -x {} \; -exec cp {} /home/citrified/executables \;

WORKDIR /home/citrified/executables

USER citrified

ENTRYPOINT ["/bin/bash"]
