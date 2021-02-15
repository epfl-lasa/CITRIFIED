ARG BASE_IMAGE=continuumio/anaconda3

FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y  \
    wget \
    git \
    vim \
    bash-completion \
    build-essential \
    sudo \
    libgl1-mesa-glx \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Now create the same user as the host itself
ENV USER anaconda
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "Anaconda User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "anaconda ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# workspace setup
WORKDIR /home/anaconda
RUN git clone https://github.com/epfl-lasa/surgeon_recording.git
COPY environment.yml ./

# conda env setup
RUN conda env create -f ./environment.yml
ENV PATH /opt/conda/envs/$(head -1 ./environment.yml | cut -d' ' -f2)/bin:$PATH

# surgeon_recording setup
RUN conda run -n $(head -1 ./environment.yml | cut -d' ' -f2) /bin/bash -c 'cd surgeon_recording/source/surgeon_recording && python setup.py install'

# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /conda_entrypoint.sh
RUN sudo chmod +x /conda_entrypoint.sh ; sudo chown anaconda /conda_entrypoint.sh ;

# Clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER anaconda
RUN echo "source activate $(head -1 ./environment.yml | cut -d' ' -f2)" > ~/.bashrc

ENTRYPOINT ["/conda_entrypoint.sh"]
CMD ["bash"]