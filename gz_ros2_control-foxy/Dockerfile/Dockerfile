FROM osrf/ros:foxy-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    wget \
    curl \
  && apt-get clean

RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
  apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-ros \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean

RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src \
    && git clone https://github.com/ignitionrobotics/ign_ros2_control/ -b foxy \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro foxy

RUN cd /home/ros2_ws/ \
  && . /opt/ros/foxy/setup.sh \
  && colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch ign_ros2_control_demos cart_example_position.launch.py
