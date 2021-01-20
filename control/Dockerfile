ARG BASE=ghcr.io/eeberhard/cpp-remote-env
ARG TAG=latest
FROM $BASE:$TAG as debug

# get base dependencies
RUN sudo apt-get install -y libtool pkg-config autoconf automake git

# get libsodium
RUN wget https://download.libsodium.org/libsodium/releases/libsodium-1.0.18.tar.gz \
  && tar -xzf libsodium-1.0.18.tar.gz \
  && cd libsodium-1.0.18 \
  && ./autogen.sh \
  && ./configure \
  && make check \
  && sudo make install \
  && sudo ldconfig \
  && cd .. \
  && rm -rf libsodium*

# get base libzmq
RUN wget https://github.com/zeromq/libzmq/releases/download/v4.3.3/zeromq-4.3.3.tar.gz \
  && tar -xzf zeromq-4.3.3.tar.gz \
  && cd zeromq-4.3.3 \
  && mkdir build \
  && cd build \
  && cmake .. -DWITH_PERF_TOOL=OFF -DZMQ_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
  && sudo make -j4 install \
  && cd ../.. \
  && rm -rf zeromq*

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

# install Eigen3
RUN git clone https://gitlab.com/libeigen/eigen.git \
  && cd eigen \
  && git checkout a36d19c4fc46097da9dd9965e7d548aa05d2cbf8 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make install \
  && cd ../.. \
  && rm -rf eigen*

# install modulo
RUN git clone https://github.com/epfl-lasa/modulo.git \
  && cd modulo/source/lib \
  && ./build.sh \
  && cd ../../.. \
  && rm -rf modulo*

# "install" passive-ds-control repo (we will build it in our own makefile)
RUN git clone https://github.com/epfl-lasa/passive-ds-control.git \
  && cd passive-ds-control \
  && mv include /usr/local/include/passive-ds-control \
  && mv src /usr/local/src/passive-ds-control \
  && cd .. \
  && rm -rf passive-ds-control

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

WORKDIR /tmp/CITRIFIED/control/scripts
RUN sh setup.sh

WORKDIR /tmp/CITRIFIED/control/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release \
  && make \
  && mkdir -p /home/citrified/executables \
  && find executables -maxdepth 1 -type f -exec test -x {} \; -exec cp {} /home/citrified/executables \;

WORKDIR /home/citrified/executables
RUN rm -rf /tmp/CITRIFIED

USER citrified

ENTRYPOINT ["/bin/bash"]