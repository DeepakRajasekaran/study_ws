# Use the ROS2 Humble Desktop image as the base
FROM osrf/ros:humble-desktop

# Set up environment variables for convenience
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for Zsh, Oh My Zsh, and necessary plugins
RUN apt update && apt install -y \
    zsh \
    curl \
    git \
    && chsh -s $(which zsh)

# Install Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" || true

# Install Zsh plugins (autosuggestions and syntax highlighting)
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# Update the .zshrc to enable plugins
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/' ~/.zshrc

# Set up the workspace directory
WORKDIR /home/user/humble_ws

# Copy the contents of your local workspace to the Docker container
COPY . /home/user/humble_ws

# Install ROS dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source the workspace and switch to Zsh as the default shell
RUN echo "source /home/user/humble_ws/install/setup.bash" >> ~/.zshrc

# Provide a way to add new libraries easily
# Add a script where users can specify additional libraries
# COPY install_additional_libraries.sh /home/user/install_additional_libraries.sh
RUN chmod +x /home/user/install_additional_libraries.sh

# Default shell to Zsh
SHELL ["/bin/zsh", "-c"]

# Start the container with Zsh
CMD ["zsh"]
