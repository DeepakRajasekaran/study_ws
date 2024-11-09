# Use the ROS2 Humble Desktop image as the base
FROM osrf/ros:humble-desktop

# Set up environment variables for convenience
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies as root
RUN apt update && apt install -y \
    zsh \
    curl \
    git

# Create a new user 'dev' without sudo privileges
RUN useradd -m -s /bin/zsh dev

# Install Oh My Zsh for the 'dev' user
RUN su - dev -c "sh -c \"$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)\" || true"

# Install Zsh plugins (autosuggestions and syntax highlighting) for the 'dev' user
RUN su - dev -c "git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-/home/dev/.oh-my-zsh/custom}/plugins/zsh-autosuggestions" && \
    su - dev -c "git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-/home/dev/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting"

# Update .zshrc to enable plugins for the 'dev' user
RUN su - dev -c "sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' ~/.zshrc"

# Set up the workspace directory and copy the local contents
WORKDIR /home/dev/humble_ws
COPY . /home/dev/humble_ws

# Change ownership of the workspace directory to 'dev'
RUN chown -R dev:dev /home/dev

# Install ROS dependencies and build the workspace as 'dev'
USER dev
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source the workspace in .zshrc for the 'dev' user
RUN echo "source /home/dev/humble_ws/install/setup.bash" >> /home/dev/.zshrc

# Set the default shell to Zsh
SHELL ["/bin/zsh", "-c"]

# Start the container with Zsh
CMD ["zsh"]

