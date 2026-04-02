## Voraussetzungen
- Ubuntu 22.04 LTS (in UTM oder VirtualBox)
 
---

## Quick Start (Automatisierte Installation)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/yarafahr/embedded_systems_project.git

cd ~/ros2_ws/src/embedded_systems_project
chmod +x setup.sh
./setup.sh
source ~/.bashrc
```
 
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

```

### 4. Dependency Repos clonen
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.som/azeey/turtlebot3_simulations -b new_gazebo

```
 
### 5. rosdep initialisieren
```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
 
### 6. Workspace bauen
```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 7. Migriertes Burger Model injezieren
``` bash
cp ~/ros2_ws/src/embedded_systems_project/models/turtlebot3_burger/model.sdf ~/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
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
embedded_systems_project/
├── docs/                       # Dokumentation
│   └── ...
├── statecharts/                # Statecharts
│   └── ...
├── worlds/
|   ├── mars_v3.sdf             # Mars-Simulationswelt
│   └── ...
├── models/
│   └── turtlebot3_burger/
│       └── model.sdf           # migriertes burger Modell
├── nodes/
│   └── battery_monitor_v3.py   # Battery Monitor node
├── params/
│   └── nav2_params.yaml        # Parameter für Nav2
├── launch/
│   └── mars_v3_launch.py       # Startet Gazebo + TurtleBot3
└── setup.sh                    # projekt installations script
```
 
---

## SLAM und Telop

Starte die Simulation:
``` bash
ros2 launch ~/ros2_ws/src/embedded_systems_project/launch/mars_v3_launch.py
```

Starte die SLAM Node in einem anderen Terminal:
``` bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

Starte die teleop Node in einem anderen Terminal:
``` bash
ros2 run turtlebot3_teleop teleop_keyboard
```
![SLAM und Teleop Screnshot](/docs/SLAM_and_teleop/screenshot.png)

Nun kann der Roboter mit der Tastatur gesteuert werden, während er eine interne repräsentation der Welt erstellt.

Zum speichern der Karte, in einem anderen Terminal:
``` bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

resultierende map:
![interne map](/docs/SLAM_and_teleop/map.png)

## SLAM und Navigation

[reference](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

Starte die Simulation:
``` bash
ros2 launch ~/ros2_ws/src/embedded_systems_project/launch/mars_v3_launch.py
```

Starte SLAM-toolbox in einem anderen Terminal:
``` bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

```

Starte Nav2 in einem anderen Terminal:
``` bash
ros2 launch nav2_bringup navigation_launch.py   use_sim_time:=True   params_file:=$HOME/ros2_ws/src/embedded_systems_project/params/nav2_params.yaml
```

Starte RViz in einem anderen Terminal:
``` bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Nun Kannst du in RVIZ "Nav2Goal" anklicken, auf der Karte das Ziel markieren und der Roboter findet autonom dahin.

Nach dem Start:
![after_bringup](/docs/SLAM_and_navigation/Screenshot_bringup.png)

Während dem navigieren:
![while_navigating](/docs/SLAM_and_navigation/Screenshot_navigating.png)s

Nach einiger Zeit des (manuellen) Erkunden:
![after_exploring](/docs/SLAM_and_navigation/Screenshot_after_exploring.png)


## SLAM, Navigation und Battery


Starte die Simulation:
``` bash
ros2 launch ~/ros2_ws/src/embedded_systems_project/launch/mars_v3_launch.py
```

Starte SLAM-toolbox in einem anderen Terminal:
``` bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

```

Starte Nav2 in einem anderen Terminal:
``` bash
ros2 launch nav2_bringup navigation_launch.py   use_sim_time:=True   params_file:=$HOME/ros2_ws/src/embedded_systems_project/params/nav2_params.yaml
```

Starte RViz in einem anderen Terminal:
``` bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Starte Battery_monitor in einem anderen Terminal:
``` bash
python3 ~/ros2_ws/src/embedded_systems_project/nodes/battery_monitor_v3.py
```

Mit diessen kommandos kannst du dir die Events des Roboters ansehen:
```bash
ros2 topic echo /Wifi/send_position
ros2 topic echo /status
```