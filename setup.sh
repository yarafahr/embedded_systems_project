#!/bin/bash

# TurtleBot3 Mars Simulation - Automated Installation Script
# This script sets up the complete ROS 2 Humble workspace with all dependencies
# Following official ROS 2 Humble installation guide

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu 22.04
check_ubuntu_version() {
    print_status "Checking Ubuntu version..."
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_ID" != "22.04" ]; then
            print_warning "This script is designed for Ubuntu 22.04 LTS. You are running $VERSION_ID."
            read -p "Continue anyway? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    fi
}

# Check if ROS 2 Humble is already installed
check_ros2_installation() {
    print_status "Checking for ROS 2 Humble installation..."
    if [ -f /opt/ros/humble/setup.bash ]; then
        print_status "ROS 2 Humble is already installed."
        return 0
    else
        print_warning "ROS 2 Humble is not installed."
        return 1
    fi
}

# Set locale (official step 1)
set_locale() {
    print_status "Setting up locale..."
    
    locale  # check for UTF-8
    
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    locale  # verify settings
    
    print_status "Locale configured."
}

# Setup Sources (official step 2)
setup_sources() {
    print_status "Setting up ROS 2 apt sources..."
    
    # Ensure Ubuntu Universe repository is enabled
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    
    # Install ros2-apt-source package (official method)
    sudo apt update && sudo apt install -y curl
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    
    print_status "ROS 2 apt sources configured."
}

# Install ROS 2 packages (official step 3)
install_ros2() {
    print_status "Installing ROS 2 Humble..."
    
    # Update apt repository caches
    sudo apt update
    
    # Upgrade system (IMPORTANT: addresses systemd/udev issue)
    print_status "Upgrading system packages (required to avoid systemd issues)..."
    sudo apt upgrade -y
    
    # Install ROS 2 Desktop (Recommended)
    print_status "Installing ros-humble-desktop..."
    sudo apt install -y ros-humble-desktop
    
    # Install development tools
    print_status "Installing ros-dev-tools..."
    sudo apt install -y ros-dev-tools
    
    print_status "ROS 2 Humble installed successfully."
}

# Install required packages
install_packages() {
    print_status "Installing required packages..."
    
    sudo apt update
    sudo apt install -y \
        ros-humble-turtlebot3* \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-rosbridge-suite \
        ros-humble-turtlebot3-description \
        ros-humble-robot-state-publisher \
        ignition-fortress \
        ros-humble-ros-ign-bridge \
        ros-humble-ros-ign-gazebo \
        python3-colcon-common-extensions \
        python3-rosdep \
        git
    
    print_status "All packages installed successfully."
}

# Clone dependency repositories
clone_dependencies() {
    print_status "Cloning dependency repositories..."
    
    cd ~/ros2_ws/src
    
    # Check and clone DynamixelSDK
    if [ ! -d "DynamixelSDK" ]; then
        print_status "Cloning DynamixelSDK..."
        git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    else
        print_status "DynamixelSDK already exists, skipping..."
    fi
    
    # Check and clone turtlebot3_msgs
    if [ ! -d "turtlebot3_msgs" ]; then
        print_status "Cloning turtlebot3_msgs..."
        git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    else
        print_status "turtlebot3_msgs already exists, skipping..."
    fi
    
    # Check and clone turtlebot3
    if [ ! -d "turtlebot3" ]; then
        print_status "Cloning turtlebot3..."
        git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
    else
        print_status "turtlebot3 already exists, skipping..."
    fi
    
    # Check and clone turtlebot3_simulations
    if [ ! -d "turtlebot3_simulations" ]; then
        print_status "Cloning turtlebot3_simulations (new_gazebo branch)..."
        git clone https://github.com/azeey/turtlebot3_simulations.git -b new_gazebo
    else
        print_status "turtlebot3_simulations already exists, skipping..."
    fi
    
    print_status "All dependencies cloned successfully."
}

# Initialize and update rosdep
setup_rosdep() {
    print_status "Setting up rosdep..."
    
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    else
        print_status "rosdep already initialized."
    fi
    
    rosdep update
    
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    
    print_status "rosdep setup complete."
}

# Build the workspace
build_workspace() {
    print_status "Building ROS 2 workspace..."
    
    cd ~/ros2_ws
    colcon build
    
    print_status "Workspace built successfully."
}

# Copy updated burger model
copy_burger_model() {
    print_status "Copying updated burger model..."
    
    SOURCE_MODEL="$HOME/ros2_ws/src/embedded_systems_project/models/turtlebot3_burger/model.sdf"
    TARGET_MODEL="$HOME/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
    
    if [ -f "$SOURCE_MODEL" ]; then
        cp "$SOURCE_MODEL" "$TARGET_MODEL"
        print_status "Burger model copied successfully."
    else
        print_warning "Source model not found at $SOURCE_MODEL"
        print_warning "You may need to copy the model manually after the build completes."
    fi
}

# Setup environment variables in .bashrc
setup_environment() {
    print_status "Setting up environment variables..."
    
    # Backup .bashrc
    cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)
    
    # Check if lines already exist to avoid duplicates
    grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
    grep -qxF 'export TURTLEBOT3_MODEL=burger' ~/.bashrc || echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
    grep -qxF 'export LIBGL_ALWAYS_SOFTWARE=1' ~/.bashrc || echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
    grep -qxF 'source ~/ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
    
    print_status "Environment variables added to .bashrc"
}

# Main installation process
main() {
    echo "========================================="
    echo "TurtleBot3 Mars Simulation Setup Script"
    echo "========================================="
    echo ""
    
    check_ubuntu_version
    
    # Check if workspace directory exists
    if [ ! -d ~/ros2_ws/src/embedded_systems_project ]; then
        print_error "embedded_systems_project not found in ~/ros2_ws/src/"
        print_error "Please clone the repository first:"
        print_error "  mkdir -p ~/ros2_ws/src"
        print_error "  cd ~/ros2_ws/src"
        print_error "  git clone https://github.com/yarafahr/embedded_systems_project.git"
        exit 1
    fi
    
    # Install ROS 2 if not already installed
    if ! check_ros2_installation; then
        read -p "Install ROS 2 Humble? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            set_locale
            setup_sources
            install_ros2
        else
            print_error "ROS 2 Humble is required. Exiting."
            exit 1
        fi
    fi
    
    # Source ROS 2
    source /opt/ros/humble/setup.bash
    
    install_packages
    clone_dependencies
    setup_rosdep
    build_workspace
    copy_burger_model
    setup_environment
    
    echo ""
    echo "========================================="
    print_status "Installation completed successfully!"
    echo "========================================="
    echo ""
    echo "To start using the workspace, run:"
    echo "  source ~/.bashrc"
    echo ""
    echo "To launch the simulation, run:"
    echo "  ros2 launch ~/ros2_ws/src/embedded_systems_project/launch/mars_launch.py"
    echo ""
    echo "For teleop control (in a separate terminal):"
    echo "  ros2 run turtlebot3_teleop teleop_keyboard"
    echo ""
}

# Run main function
main