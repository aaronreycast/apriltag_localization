FROM ros:jazzy
WORKDIR /ws
COPY package.xml src/apriltag_localization/package.xml
RUN apt-get update && rosdep install --from-paths src --ignore-src -y
COPY . src/apriltag_localization
RUN . /opt/ros/jazzy/setup.sh && colcon build
CMD ["bash"]