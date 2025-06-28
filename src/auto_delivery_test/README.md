# 🤖 TurtleBot3 Auto Delivery Robot - ROS 2 Humble

A ROS 2 Humble-based autonomous delivery system using TurtleBot3 Burger and Nav2 stack in Gazebo. This project simulates a delivery robot that can move between a **home base**, **kitchen**, and multiple **tables** (1, 2, 3) based on order inputs and confirmations.

---

## 📋 Table of Contents

- [🔧 Requirements](#-requirements)
- [📦 Package Structure](#-package-structure)
- [🧠 Project Logic](#-project-logic)
- [🧪 Test Case Strategy](#-test-case-strategy)
- [🚀 Launch Instructions](#-launch-instructions)
- [📂 Folder Tree](#-folder-tree)


---

## 🔧 Requirements

| Component              | Version              |
|------------------------|----------------------|
| Ubuntu                 | 22.04 LTS            |
| ROS 2                  | Humble Hawksbill     |
| Gazebo                 | Gazebo 11            |
| TurtleBot3 Model       | burger               |
| nav2_simple_commander  | included in nav2     |
| Python Version         | 3.10                 |

Install TurtleBot3 packages and dependencies before running.

---

## 📦 Package Structure

turtlebot3_ws/
└── src/
└── auto_delivery_test/
├── auto_delivery_test/
│ └── run_all_test_cases.py
├── launch/
│ └── bringup_delivery.launch.py # Optional
├── package.xml
├── setup.py
└── README.md



---

## 🧠 Project Logic

- Robot starts from **Home Position**
- Waits for command `s` to begin delivery
- Moves to **Kitchen**
- Accepts **Table numbers** as input (e.g., `1 2 3`)
- For each table:
  - Cancels if `c` is pressed before reaching
  - Waits for food pickup confirmation (Enter)
  - Skips if timeout or cancelled
- After all deliveries:
  - If any skipped: returns to **Kitchen**, then goes **Home**
  - If all delivered: goes directly **Home**

---

## 🧪 Test Case Strategy

| Test Case | Description |
|-----------|-------------|
| **1**     | Simple delivery: Home → Kitchen → Table → Home |
| **2**     | Timeout at kitchen → Return Home |
| **3a**    | Timeout at kitchen → Return Home |
| **3b**    | Timeout at table → Return Kitchen → Enter → Home |
| **4**     | Cancel en route: Kitchen = go Home, Table = go Kitchen → Enter → Home |
| **5**     | Multiple orders: Table 2, 1 → Enter after each → Home |
| **6**     | Timeout at Table 2 → go Table 3 → go Table 1 → Kitchen → Home |
| **7**     | Cancel Table 2 → deliver to 1, 3 → Kitchen → Home |

---

## 🚀 Launch Instructions

Each command should be run in a separate terminal:

### 🔹 Terminal 1 – Gazebo World
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


### 🔹 Terminal 2 – Spawn Robot in Gazebo

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0 -y 0 -z 0.01



### 🔹 Terminal 3 – Launch Navigation2

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True



### 🔹 Terminal 4 – Run Auto Delivery Code

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run auto_delivery_test run_all_test_cases

Folder Tree (summary)

auto_delivery_test/
├── auto_delivery_test/
│   └── run_all_test_cases.py
├── launch/
│   └── bringup_delivery.launch.py  (optional)
├── setup.py
├── package.xml
└── README.md



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
