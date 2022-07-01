FROM ubuntu:20.04

# change the locale from POSIX to UTF-8
# might not be necessary
RUN apt-get update && apt-get install -y --no-install-recommends locales software-properties-common \
&& locale-gen en_US en_US.UTF-8 \
&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
&& export LANG=en_US.UTF-8 \
&& add-apt-repository universe

# add ROS deb repo to apt sources list
RUN apt-get update && apt-get install -y --no-install-recommends \
	curl \
	gnupg2 \
	lsb-release \
  build-essential
RUN apt-get update \
&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
&& sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
&& curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update \
&& apt-get upgrade -y \
&& apt-get install -y --no-install-recommends ros-foxy-ros-base \
&& apt-get install -y --no-install-recommends python3-rosdep \
&& apt-get install -y --no-install-recommends python3-colcon-common-extensions \
&& apt-get install -y --no-install-recommends ros-foxy-mavros ros-foxy-mavros-extras

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
&& bash ./install_geographiclib_datasets.sh 

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
&& rosdep init && rosdep update
