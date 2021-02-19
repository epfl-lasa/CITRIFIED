ARG BASE=ghcr.io/eeberhard/cpp-remote-env
ARG TAG=latest
FROM $BASE:$TAG as project-dependencies

# get base dependencies
RUN sudo apt-get install -y libtool pkg-config autoconf automake git libsodium-dev libzmq3-dev
RUN sudo apt-get install -y libeigen3-dev libboost-all-dev libtbb-dev libgtest-dev

RUN sudo apt-get -qq update && sudo apt-get -qq --yes --force-yes install autoconf automake
RUN wget http://members.loria.fr/JBMouret/mirrors/nlopt-2.4.2.tar.gz \
  && tar -zxvf nlopt-2.4.2.tar.gz \
  && cd nlopt-2.4.2 \
  && ./configure -with-cxx --enable-shared --without-python --without-matlab --without-octave \
  && sudo make install \
  && sudo ldconfig \
  && cd .. \
  && rm -rf nlopt-2.4.2

RUN git clone -b release-2.1 --single-branch https://github.com/resibots/limbo.git \
  && cd limbo \
  && ./waf configure \
  && ./waf build

WORKDIR /tmp
# get cppzmq bindings
RUN wget https://github.com/zeromq/cppzmq/archive/v4.7.1.tar.gz -O cppzmq-4.7.1.tar.gz \
  && tar -xzf cppzmq-4.7.1.tar.gz \
  && cd cppzmq-4.7.1 \
  && mkdir build \
  && cd build \
  && cmake .. -DCPPZMQ_BUILD_TESTS=OFF \
  && make -j4 install

# install control libraries
RUN git clone -b develop --single-branch https://github.com/epfl-lasa/control_libraries.git \
  && mkdir -p control_libraries/source/build \
  && cd control_libraries/source/build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_ROBOT_MODEL="OFF" .. \
  && make -j \
  && make install

# install KDL
RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git \
  && cd orocos_kinematics_dynamics/orocos_kdl \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install

# install yaml-cpp
RUN git clone https://github.com/jbeder/yaml-cpp.git \
  && cd yaml-cpp \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install

COPY ./good_standardized_sampled_train.csv /tmp/train.csv

# install json library header
RUN wget https://github.com/nlohmann/json/releases/download/v3.9.1/json.hpp \
  && mv json.hpp /usr/local/include

# install gtest
RUN mkdir -p /tmp/gtest_build \
  && cd /tmp/gtest_build \
  && cmake /usr/src/gtest \
  && make \
  && cp lib/* /usr/local/lib || cp *.a /usr/local/lib

# clean up installation artefacts
WORKDIR /
RUN rm -rf /tmp/*
RUN ldconfig

# tag a new build stage that copies the project source files
FROM project-dependencies as project-sources

WORKDIR /usr/src/CITRIFIED/control
COPY executables executables
COPY include include
COPY lib lib
COPY scripts scripts
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
