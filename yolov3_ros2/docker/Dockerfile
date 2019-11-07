FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

SHELL ["/bin/bash", "-c"]

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -q -y \
        curl \
        gnupg2 \
        lsb-release \
        wget \
        libsm6 \
        libxext6 \
        libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl http://repo.ros2.org/repos.key | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install --no-install-recommends -y \
        git \
        python3-pip \
        python3-rospkg \
        python3-catkin-pkg \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U \
        argcomplete \
        setuptools \
        opencv-python \
        scikit-learn \
        scikit-image \
        tensorflow-gpu \
        keras \
        cython \
        cffi

RUN apt-get update && apt-get install -y \
        ros-dashing-ros-base \
        ros-dashing-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/dashing/setup.bash" >> /root/.bashrc

WORKDIR /root

RUN mkdir -p ros2_ws/src

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"

RUN echo 'export PATH=/usr/local/cuda/bin:${PATH}' >> ~/.bashrc
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc

# set default python vertion 3
RUN rm -f /usr/bin/python
RUN ln -s /usr/bin/python3 /usr/bin/python

WORKDIR /root

COPY ./ros_entrypoint.sh /
CMD ["/ros_entrypoint.sh"]
