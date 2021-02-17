ARG BASE=ghcr.io/eeberhard/cpp-remote-env
ARG TAG=latest
FROM $BASE:$TAG as project-dependencies

# get base dependencies
RUN sudo apt-get install -y libtool pkg-config autoconf automake git \
  libsodium-dev libzmq3-dev libgtest-dev libboost-all-dev

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

# install gtest
WORKDIR /tmp/gtest_build
RUN cmake /usr/src/gtest \
  && sudo make \
  && cp lib/* /usr/local/lib

# install yaml-cpp
WORKDIR /tmp
RUN git clone https://github.com/jbeder/yaml-cpp.git \
  && cd yaml-cpp \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install

# tag a new build stage that copies the project source files
FROM project-dependencies as project-sources

WORKDIR /usr/src/CITRIFIED/control
COPY executables executables
COPY include include
COPY lib lib
COPY source source
COPY CMakeLists.txt CMakeLists.txt


# tag a new build stage that compiles and runs the test targets
FROM project-sources as tester

WORKDIR /usr/src/CITRIFIED/control/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=True \
  && make
RUN make test


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
