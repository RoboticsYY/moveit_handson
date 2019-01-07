# Dockerfile to build moveit_handson container images
# Example build command:
# sudo docker build -t intel/kinetic:moveit_handson --build-arg http_proxy=http://my.proxy.com:### .
FROM osrf/ros:kinetic-desktop

SHELL ["/bin/bash", "-c"]

# setup keys
# check if proxy is set and get keys, using proxy if it is set
RUN if [ "$http_proxy" == "" ]; \
    then \
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116; \
    else \
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --keyserver-options http-proxy=$http_proxy \
      --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116; \
fi

# update latest package versions
RUN apt-get update

# Install tools
RUN apt-get install -y wget \
    git \
    sudo \
    cpio \
    make \
    cmake \
    cppcheck \
    python-pip

# Install opencv
RUN mkdir -p /root/code/opencv
WORKDIR /root/code/opencv
RUN git clone -b 3.3.0 https://github.com/opencv/opencv
RUN git clone -b 3.3.0 https://github.com/opencv/opencv_contrib
RUN mkdir -p opencv/build
WORKDIR /root/code/opencv/opencv/build
RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_cnn_3dobj=OFF ../
RUN make -j$(nproc) && make install
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf
RUN ldconfig

# Install moveit
RUN apt-get install -y ros-kinetic-moveit
RUN apt-get install -y ros-kinetic-moveit-resources
RUN apt-get install -y ros-kinetic-moveit-visual-tools
RUN apt-get install -y ros-kinetic-panda-moveit-config
RUN apt-get install -y ros-kinetic-geometric-shapes

# Install necessary debian packages
RUN apt install -y python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential
RUN apt-get install -y python-wstool \
    python-catkin-tools \
    clang-format-3.8
RUN apt-get install -y ros-kinetic-universal-robot
RUN apt-get install -y ros-kinetic-ur-description
RUN apt-get install -y ros-kinetic-tf2-geometry-msgs

# Install handson source code:
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN mkdir -p /root/ws_handson/src
WORKDIR  /root/ws_handson/src
RUN git clone https://github.com/RoboticsYY/moveit_handson.git
RUN git clone https://github.com/RoboticsYY/moveit_core_handson.git
RUN git clone https://github.com/RoboticsYY/moveit_ros_handson.git
WORKDIR /root/ws_handson
RUN ( . /opt/ros/kinetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release)

CMD ["bash"]
