FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1

RUN apt update && \
    apt install -y \
        build-essential \
        cmake \
        git \
        curl lsb-release gnupg \
        libbullet-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-colcon-clean \
        ros-dev-tools \
        ros-humble-apriltag-ros \
        ros-humble-foxglove-bridge \
        ros-humble-mavros \
        ros-humble-mavros-extras \
        ros-humble-ros2bag \
        ros-humble-rosbag2* \
        libeigen3-dev \
        libboost-all-dev \
        libceres-dev \
        wget \
        clang-format && \
    python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest

RUN pip3 install \
    kconfiglib \
    jsonschema \
    jinja2 \
    pyros-genmsg \
    future 


# Install Gazebo Harmonic - Default Gazebo version in ROS2 Humble is Fortress
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update

# Build ros_gz from source (as of 12-08-25 the prebuilt binaries do not include translation of LaserScan messages into Range ROS messages)
ENV GZ_VERSION=harmonic

# Initialize rosdep as root (ignore if already initialized)
RUN rosdep init || true

RUN useradd -rm -d /home/developer -s /bin/bash -g root -G sudo -u 1001 developer && \
    echo 'developer:developer' | chpasswd && \
    echo 'developer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer

RUN apt-get install -y geographiclib-tools && \
        geographiclib-get-geoids egm96-5

USER developer
RUN git clone --branch v1.16.0-rc3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --recursive
RUN echo developer | sudo -S bash /home/developer/PX4-Autopilot/Tools/setup/ubuntu.sh


# ---- ros_gz from source (humble branch) ----
# Create a colcon workspace and clone ros_gz
RUN mkdir -p /home/developer/ws/src && \
    cd /home/developer/ws/src && \
    git clone https://github.com/gazebosim/ros_gz.git -b humble

# Install dependencies (this may also install Gazebo libraries for Harmonic)
RUN cd /home/developer/ws && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro "$ROS_DISTRO"

# Build the workspace
RUN bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && cd /home/developer/ws && colcon build"

# Source the overlay on shell startup
RUN echo ". /home/developer/ws/install/setup.bash" >> ~/.bashrc
# ---- end ros_gz from source ----

# Install OpenVINS
RUN mkdir -p ~/workspace/catkin_ws_ov/src/ && \
    cd ~/workspace/catkin_ws_ov/src/ && \
    git clone https://github.com/rpng/open_vins/ && \
    cd .. && \
    . /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

COPY . /home/developer/ros2_ws
WORKDIR /home/developer/ros2_ws

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo ". /home/developer/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && echo ". ../workspace/catkin_ws_ov/install/setup.bash" >> ~/.bashrc

CMD [ "sleep", "infinity" ]
