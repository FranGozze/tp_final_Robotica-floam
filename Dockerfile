FROM osrf/ros:noetic-desktop

RUN apt-get update && \
    apt-get install -y  \
        git \
        ros-noetic-hector-trajectory-server
RUN apt-get update && \
    apt-get install -y  \
        cmake \
        libgoogle-glog-dev libgflags-dev \
        libatlas-base-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        libpcl-dev && \
    rm -rf /var/lib/apt/lists/*


RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git submodule update --init --recursive  && \
    cd ..

RUN  apt-get update && \
    apt-get install -y \
    build-essential \
    libopenblas-dev \
    liblapack-dev

RUN mkdir ceres-bin && \
    cd ceres-bin && \
    cmake ../ceres-solver -DBUILD_TESTING=OFF && \
    make -j3 && \
    make install
        
RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-perception-pcl

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/wh200720041/floam.git && \
    cd .. && \
    catkin_make
    
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
# ENTRYPOINT ["/ros_entrypoint.sh"]
# Arranca en bash por defecto (podés sobreescribir con `docker run ... <cmd>`)
CMD ["bash"]