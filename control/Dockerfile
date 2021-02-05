ARG BASE=ghcr.io/eeberhard/cpp-remote-env
ARG TAG=latest
FROM $BASE:$TAG as debug

# get base dependencies
RUN sudo apt-get install -y libtool pkg-config autoconf automake git libsodium-dev libzmq3-dev

# get cppzmq bindings
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
  && tar -xzf cppzmq-4.7.1.tar.gz \
  && cd cppzmq-4.7.1 \
  && mkdir build \
  && cd build \
  && cmake .. -DCPPZMQ_BUILD_TESTS=OFF \
  && sudo make -j4 install \
  && cd ../.. \
  && rm -rf cppzmq*

# install control libraries
RUN git clone -b develop --single-branch https://github.com/epfl-lasa/control_libraries.git
WORKDIR control_libraries/source
RUN /bin/bash -c "source ./install.sh"
RUN rm -rf control_libraries*

# "install" passive-ds-control repo (we will build it in our own makefile)
RUN git clone https://github.com/epfl-lasa/passive-ds-control.git \
  && cd passive-ds-control \
  && mv include /usr/local/include/passive-ds-control \
  && mv src /usr/local/src/passive-ds-control \
  && cd .. \
  && rm -rf passive-ds-control

# install KDL
RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git \
  && cd orocos_kinematics_dynamics/orocos_kdl \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install \
  && cd ../../.. \
  && rm -rf orocos_kinematics_dynamics

RUN sudo apt-get install -y libboost-all-dev

# tag a new build stage that compiles the sources
FROM debug as runtime

RUN useradd -ms /bin/bash citrified

RUN sudo apt-get install -y curl

WORKDIR /tmp/CITRIFIED/control
COPY executables executables
COPY include include
COPY lib lib
COPY scripts scripts
COPY source source
COPY CMakeLists.txt CMakeLists.txt

WORKDIR /tmp/CITRIFIED/control/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
  && make \
  && mkdir -p /home/citrified/executables \
  && find executables -maxdepth 1 -type f -exec test -x {} \; -exec cp {} /home/citrified/executables \;

WORKDIR /home/citrified/executables
RUN rm -rf /tmp/CITRIFIED

USER citrified

ENTRYPOINT ["/bin/bash"]
