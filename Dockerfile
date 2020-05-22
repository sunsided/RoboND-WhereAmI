FROM sunside/ros-gazebo-gpu:kinetic-nvidia

USER root

# Install required ROS packages, as well as SSH for Remote Host development.
RUN apt-get update && apt-get install -y \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-effort-controllers \
    ros-kinetic-joint-state-controller \
    gdb \
 && rm -rf /var/lib/apt/lists/*

# Allow SSH login into the container.
# See e.g. https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Start SSH server and run bash as the ros user.
USER ros
ENTRYPOINT sudo service ssh restart && bash