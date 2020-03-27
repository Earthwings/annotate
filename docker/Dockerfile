FROM ros:kinetic-ros-base

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

COPY NVIDIA-DRIVER.run /tmp/NVIDIA-DRIVER.run

# Configure apt and install packages
RUN apt-get update \
    && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
    #
    # Install Nvidia drivers related tools
    && apt-get -y install kmod mesa-utils binutils \
    # 
    # Verify git, process tools, lsb-release (useful for CLI installs) installed
    && apt-get -y install git iproute2 procps lsb-release \
    #
    # Install C++ tools
    && apt-get -y install build-essential cmake cppcheck valgrind \
    # Install ROS packages
    && apt-get -y install python-catkin-tools ros-kinetic-rviz \
    # libqt5widgets5 libqt5gui5 qtbase5-dev libogre-1.9-dev liburdfdom-dev
    #
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/user/.bashrc \
    #
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    #
    # Install nvidia driver
    && sh /tmp/NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module && rm /tmp/NVIDIA-DRIVER.run \
    && ln -s /usr/lib/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog

USER user
ENTRYPOINT ["/ros_entrypoint.sh"]
