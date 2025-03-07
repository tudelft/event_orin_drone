
# orin: BASE_IMAGE=event-orin:r36.3.0
# x86: BASE_IMAGE=nvidia/cuda:12.1.0-devel-ubuntu22.04

ARG BASE_IMAGE=nvidia/cuda:12.1.0-devel-ubuntu22.04
FROM $BASE_IMAGE

ARG BASE_IMAGE
ARG WS_SOURCE=.
ARG WS_TARGET=/workspace
ARG ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# timezone (to prevent tzdata dialog)
ENV TZ=Europe/Brussels
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# upgrade and install packages
RUN apt update \
    && apt upgrade -y --no-install-recommends \
    && apt install -y --no-install-recommends locales software-properties-common curl sudo vim git ssh bash-completion \
    python-is-python3 python3-pip python3-dev python3-venv ninja-build libatlas-base-dev
RUN if [ "$BASE_IMAGE" = "nvidia/cuda:12.1.0-devel-ubuntu22.04" ] ; then apt update && apt install -y --no-install-recommends python3-tk ; fi

# install ros (would be nice if we could merge container with ros:humble-ros-base-jammy)
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
# ros
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y ros-$ROS_VERSION-ros-base ros-dev-tools

# install realsense lib
# here instead of in jetson-containers to avoid fixes and have it on x86
# from https://github.com/dusty-nv/jetson-containers/blob/master/packages/hardware/realsense/Dockerfile
RUN apt update && apt install -y --no-install-recommends \
    libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev qtcreator udev python2-dev
ARG LIBREALSENSE_VERSION=master
RUN git clone --branch ${LIBREALSENSE_VERSION} --depth=1 https://github.com/IntelRealSense/librealsense \
    && cd librealsense \
    && mkdir build \
    && cd build \
    && cmake \
        -DBUILD_EXAMPLES=true \
	    -DFORCE_RSUSB_BACKEND=true \
	    -DBUILD_WITH_CUDA=true \
        -DCMAKE_BUILD_TYPE=release \
        -DBUILD_PYTHON_BINDINGS=bool:true \
        -DPYTHON_EXECUTABLE=/usr/bin/python \
        -DPYTHON_INSTALL_DIR=$(python -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') \
        ../ \
    && make -j$(($(nproc)-1)) \
    && make install \
    && cd ../ \
    && cp config/99-realsense-libusb.rules /etc/udev/rules.d/
RUN rm -rf /realsense
RUN pip install pyrealsense2
RUN python -c 'import pyrealsense2'

# install rosdeps
RUN apt update && rosdep init && rosdep update --rosdistro $ROS_VERSION
RUN --mount=type=bind,source=$WS_SOURCE,target=$WS_TARGET rosdep install --from-paths $WS_TARGET/ros/src --ignore-src --rosdistro $ROS_VERSION -y

# install realsense ros wrapper
# for necessary fixes, see readme
RUN apt update && apt install -y --no-install-recommends \
    ros-$ROS_VERSION-realsense2-*

# install python packages; ssh key to clone private repos
# https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials
# torch manually before requirements if x86, needed to build cuda extensions
RUN mkdir -p ~/.ssh && ssh-keyscan github.com > ~/.ssh/known_hosts
RUN if [ "$BASE_IMAGE" = "nvidia/cuda:12.1.0-devel-ubuntu22.04" ] ; then pip install torch==2.4.* torchvision==0.19.* ; fi
RUN --mount=type=ssh,uid=1000 --mount=type=bind,source=$WS_SOURCE,target=$WS_TARGET \
    pip install -U -r $WS_TARGET/docker/requirements.txt

# clean up
RUN apt autoremove -y && rm -rf /var/lib/apt/lists/* && apt clean

# user to match current user
# create but don't use it yet
# use when running container with --user $(id -u):$(id -g)
# https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user
ARG USERNAME=mavlab
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# USER $USERNAME

# custom entrypoint that sources the workspace
COPY $WS_SOURCE/docker/entrypoint.sh /entrypoint.sh

WORKDIR $WS_TARGET
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
