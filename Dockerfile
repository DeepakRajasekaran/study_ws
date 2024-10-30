# Use ROS 2 Humble Desktop as the base image
FROM ros:humble-desktop

# Install additional dependencies, including zsh, libmodbus, colcon extensions, and openssh-server
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    libmodbus-dev \
    zsh \
    zsh-syntax-highlighting \
    zsh-autosuggestions \
    curl \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# Install Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Set zsh as the default shell
RUN chsh -s /usr/bin/zsh

# Configure Oh My Zsh with plugins for syntax highlighting and autosuggestions
RUN echo "source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh" >> ~/.zshrc && \
    echo "source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh" >> ~/.zshrc && \
    echo "autoload -U compinit && compinit" >> ~/.zshrc

# Configure SSH
RUN mkdir /var/run/sshd && \
    echo 'root:password' | changeme && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "export VISIBLE=now" >> /etc/profile

# Expose the SSH port
EXPOSE 22

# Set the working directory
WORKDIR /workspace

# Copy the entire humble_ws folder into the container
COPY . /workspace/humble_ws

# Source ROS 2 setup and build all workspaces within humble_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.zsh && \
                  colcon build --base-paths /workspace/humble_ws"

# Start SSH and enter zsh shell on container start
CMD ["/bin/zsh", "-c", "/usr/sbin/sshd && source /opt/ros/humble/setup.zsh && source /workspace/humble_ws/install/setup.zsh && zsh"]
