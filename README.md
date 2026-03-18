## Voraussetzungen
- Ubuntu 22.04 LTS (in UTM oder VirtualBox)
- ROS 2 Humble
- Gazebo Fortress (Ignition)
 
---
 
## Installation
 
### 1. ROS 2 Humble installieren
Folge der offiziellen Anleitung: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
 
Danach:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```
 
### 2. Pakete installieren
```bash
sudo apt install ros-humble-turtlebot3* -y
sudo apt install ros-humble-slam-toolbox -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-rosbridge-suite -y
sudo apt install ros-humble-turtlebot3-description -y
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install ignition-fortress -y
sudo apt install ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```
 
### 3. Repo klonen
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yarafahr/embedded_systems_project.git
cd ~/ros2_ws
```
 
### 4. rosdep initialisieren
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
 
### 5. Workspace bauen
```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
 
---
 
## Simulation starten
 
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ~/ros2_ws/src/embedded_systems_project/launch/mars_launch.py
```
 
Gazebo öffnet sich mit der Mars-Welt und der TurtleBot3 wird automatisch gespawnt.
 
---
 
## Projektstruktur
 
```
mars_world/
├── worlds/
│   └── mars.sdf        # Mars-Simulationswelt
└── launch/
    └── mars_launch.py  # Startet Gazebo + TurtleBot3
```
 
---
