# 🧠 Auto Delivery Test — TurtleBot3 ROS 2 Navigation

This package implements an autonomous delivery robot simulation using **ROS 2 Humble**, **Nav2**, and **TurtleBot3 Burger** in **Gazebo**.  
The robot performs intelligent delivery operations to customer tables based on input, timeouts, and cancellations.

---

## 📦 System Requirements

- Ubuntu 22.04  
- ROS 2 Humble Hawksbill  
- TurtleBot3 (Burger model)  
- Gazebo 11  
- nav2_simple_commander  
- nav2_bringup  
- Navigation2 stack (Nav2)

---

## 📁 Folder Structure

turtlebot3_ws/
├── src/
│ └── auto_delivery_test/
│ ├── auto_delivery_test/
│ │ └── run_all_test_cases.py
│ ├── package.xml
│ ├── setup.py
│ ├── README.md
├── build/ ← after build
├── install/ ← after build
└── log/ ← after build



---

## 🚀 How to Build

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash


## Terminal 1 – Launch Gazebo World

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


## Terminal 2 – Spawn Robot into Gazebo

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash

ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0 -y 0 -z 0.01


## Terminal 3 – Launch Nav2 Navigation Stack

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

## Terminal 4 – Run Auto Delivery Logic

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

ros2 run auto_delivery_test run_all_test_cases



Behavior & Test Case Summary
Test Case	Description
1	Home → Kitchen → Table → Wait for Enter → Home
2	Home → Kitchen → Wait max 60s for Table input → If no input → Home
3a	Same as TC2
3b	Kitchen → Table → Timeout → Return to Kitchen → Wait Enter → Home
4	Press c during move → Cancel: return to Home/Kitchen depending on when canceled
5	Multiple tables: Go one by one → Wait Enter → Return to Home
6	Skip tables on timeout → Go to kitchen after last → Home
7	Cancel individual table in multi-order → Skip → Kitchen → Home


Package Dependencies

Install dependencies if not already available:

sudo apt update
sudo apt install \
  ros-humble-nav2-simple-commander \
  ros-humble-turtlebot3* \
  ros-humble-gazebo-ros \
  python3-tf-transformations


© 2025 – Arshath
GitHub: github.com/itsarshathm
