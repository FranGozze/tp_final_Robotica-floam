FROM osrf/ros:melodic-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y  \
        git \
        ros-melodic-hector-trajectory-server \
        cmake \
        libgoogle-glog-dev libgflags-dev \
        libatlas-base-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        build-essential \
        libopenblas-dev \
        liblapack-dev \
        ros-melodic-pcl-conversions \
        ros-melodic-pcl-ros \
        ros-melodic-perception-pcl \
        unzip \
        nano \
        libpcl-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch 2.1.0 https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git submodule update --init --recursive  && \
    cd ..
    
RUN mkdir ceres-bin && \
    cd ceres-bin && \
    cmake ../ceres-solver -DBUILD_TESTING=OFF && \
    make -j3 && \
    make install

RUN source /opt/ros/melodic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/RuanJY/M1_floam.git && \
    cd .. && \
    catkin_make
    
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p ~/Downloads
# ENTRYPOINT ["/ros_entrypoint.sh"]
# Arranca en bash por defecto (podés sobreescribir con `docker run ... <cmd>`)
CMD ["bash"]
    