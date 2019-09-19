FROM ubuntu:18.04

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
# Inform scripts that no questions should be asked
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Allow installing resolvconf (https://stackoverflow.com/a/45908415)
RUN echo "resolvconf resolvconf/linkify-resolvconf boolean false" | debconf-set-selections

RUN apt-get update -qq && apt-get install -qq --assume-yes --no-install-recommends apt-utils git wget curl python-psutil ssh locales locales-all udev gnupg2 lsb-release sudo ca-certificates
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install commands used in our scripts and standard present on a clean ubuntu installation
RUN apt-get update -qq && apt-get install -qq --assume-yes --no-install-recommends sudo \
vim \
build-essential \
cmake \
git \
python3-colcon-common-extensions \
python3-pip \
python-rosdep \
python3-vcstool \
python3-argcomplete \
wget \
libasio-dev \
libtinyxml2-dev

RUN python3 -m pip install -U \
argcomplete \
flake8 \
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
pytest \
pytest-cov \
pytest-runner \
setuptools

# Set Locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Make ssh dir
RUN mkdir /root/.ssh/

# Add noblean user
RUN adduser --disabled-password --gecos "" noblean
RUN adduser noblean sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER noblean
WORKDIR /home/noblean

RUN bash -c 'mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos && vcs import src < ros2.repos'

RUN bash -c 'cd ~/ros2_ws && sudo -E rosdep init && sudo -E rosdep update && sudo -E rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"'

RUN bash -c 'cd ~/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install'

# Entrypoint setup
COPY ./ros_entrypoint.sh /
RUN sudo chmod a+x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
