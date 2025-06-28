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
- [📜 Code Explanation] run_all_test_cases.py


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

📍 What happens:
This will open Gazebo with the TurtleBot3 simulation environment. You'll see a basic world with your robot placed.


### 🔹 Terminal 2 – Spawn Robot in Gazebo

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0 -y 0 -z 0.01

📍 What happens:
Spawns the TurtleBot3 robot into the Gazebo world at coordinate (0, 0). Now your robot is visible in the simulation.


### 🔹 Terminal 3 – Launch Navigation2

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

📍 What happens:

    This launches the Nav2 navigation stack, which includes the AMCL localization, path planner, behavior servers, and RViz interface.

    The robot is now waiting to know where it is on the map (initial pose).

🎯 Important Step: Set 2D Pose Estimate in RViz

Once RViz opens:

    ✅ Click “2D Pose Estimate” on the top toolbar in RViz.

    🖱️ Click anywhere near the robot's starting location on the map (usually (0, 0)), and drag slightly to set orientation (you'll see an arrow).

    🧭 This gives the AMCL (Adaptive Monte Carlo Localization) system a starting estimate of where the robot is in the world.

🟢 After this, the robot is now fully localized and ready to receive navigation goals through code (goToPose() from your script) or manually (Nav Goal in RViz).

### 🔹 Terminal 4 – Run Auto Delivery Code

export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run auto_delivery_test run_all_test_cases

📍 What happens:
Your delivery robot waits for command 's' to start. Based on your inputs, it navigates to the kitchen, accepts table numbers, performs deliveries, handles timeouts, and returns home as per all 7 test cases.

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


📜 Code Explanation: run_all_test_cases.py

This script is the heart of the autonomous delivery logic. It’s a ROS 2 Python node that controls a TurtleBot3 Burger robot to simulate a smart food delivery robot in a restaurant-like setup — using rclpy and the nav2_simple_commander interface.

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import sys
import select

# Define key positions
HOME = [0.0, 0.0]
KITCHEN = [1.0, 0.0]
TABLES = {
    '1': [1.0, 1.0],
    '2': [0.0, 1.0],
    '3': [-1.0, 1.0]
}

class AutoDeliveryBot(Node):
    def __init__(self):
        super().__init__('auto_nav_test_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("✅ Nav2 is ready. Waiting for 's' to start the delivery loop...")
        self.run_loop()

    def set_goal(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def go_to(self, label, x, y):
        self.get_logger().info(f"🚗 Navigating to {label}: ({x}, {y})")
        self.navigator.goToPose(self.set_goal(x, y))
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ Reached {label}")
            return True
        else:
            self.get_logger().warn(f"❌ Failed to reach {label}")
            return False

    def wait_for_input(self, prompt, timeout=60):
        self.get_logger().info(prompt)
        sys.stdout.flush()
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.readline().strip()
        else:
            return None

    def run_loop(self):
        while rclpy.ok():
            key = self.wait_for_input("▶️  Press 's' to start delivery or Ctrl+C to exit:", None)
            if key != 's':
                continue

            cancel = self.wait_for_input("📦 On the way to kitchen... Press 'c' to cancel or Enter to continue:", 3)
            if cancel == 'c':
                self.get_logger().warn("🚫 Cancelled before reaching kitchen. Going back home.")
                self.go_to("Home", *HOME)
                continue

            if not self.go_to("Kitchen", *KITCHEN):
                continue

            tables_input = self.wait_for_input("🍽️  Enter table numbers (e.g., 1 or 1 2 3): ", 60)
            if not tables_input:
                self.get_logger().warn("⚠️ No input. Returning home.")
                self.go_to("Home", *HOME)
                continue

            table_list = [t for t in tables_input.split() if t in TABLES]
            if not table_list:
                self.get_logger().warn("⚠️ Invalid table input. Returning home.")
                self.go_to("Home", *HOME)
                continue

            skipped_tables = []
            for table in table_list:
                pos = TABLES[table]

                if not self.go_to(f"Table {table}", *pos):
                    skipped_tables.append(table)
                    continue

                confirm = self.wait_for_input(
                    f"[Table {table}] 🧍 Press Enter after taking your food (or any key to cancel, 60s):", 60)

                if confirm != "":
                    self.get_logger().warn(f"🚫 Cancelled at Table {table}. Will return item to kitchen.")
                    skipped_tables.append(table)
                    continue

            if skipped_tables:
                self.get_logger().info("🔁 Going to kitchen to return skipped items...")
                if self.go_to("Kitchen", *KITCHEN):
                    chef = self.wait_for_input("👨‍🍳 Chef: Press Enter to take back food.", None)
                    self.get_logger().info("🍽️  Items returned to chef.")

            self.get_logger().info("🏠 Returning to home...")
            self.go_to("Home", *HOME)
            self.get_logger().info("✅ Delivery complete. Waiting for new order.\n")

def main():
    rclpy.init()
    AutoDeliveryBot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


✅ What This Code Does

Once the system is launched and ready:

    The robot starts from a home position

    It waits for the user to press 's' to begin

    Then it moves to the kitchen area

    Accepts one or more table numbers (1, 2, or 3)

    Delivers food to each table with confirmation from the user

    Handles timeouts, cancellations, and skipped tables

    Returns to the kitchen to return any undelivered food

    Finally, moves back to home

It supports both single and multi-table orders and is interactive in real-time with the user in the terminal.

📁 Main Components
1. Defined Positions in the World

HOME = [0.0, 0.0]
KITCHEN = [1.0, 0.0]
TABLES = {
    '1': [1.0, 1.0],
    '2': [0.0, 1.0],
    '3': [-1.0, 1.0]
}

These coordinates represent the fixed positions in the Gazebo simulation — the robot uses them as navigation goals.
2. AutoDeliveryBot Class

This ROS 2 node:

    Initializes Nav2 (BasicNavigator)

    Waits for it to become active

    Loops continuously to receive delivery commands

    Manages the full delivery workflow from start to finish

3. Navigation Setup

self.navigator = BasicNavigator()
self.navigator.waitUntilNav2Active()

The robot waits for the navigation system to fully activate before proceeding to goals.
4. Sending a Navigation Goal

def go_to(self, label, x, y):

This method sends the robot to a specified location. It blocks until the goal is completed and logs success or failure.
5. User Input Handling

def wait_for_input(self, prompt, timeout=60):

This method displays a prompt in the terminal and waits for input within a timeout window. It’s used for confirming or cancelling deliveries.
6. Main Logic: run_loop()

The full delivery logic is implemented here.

    Start Trigger:

key = self.wait_for_input("Press 's' to start...")

Waits for the user to press 's' before doing anything.

Cancel on the way to Kitchen:

cancel = self.wait_for_input("Press 'c' to cancel...", 3)

If the user presses 'c' quickly, the robot returns home without going to the kitchen.

Getting Table Numbers:

tables_input = self.wait_for_input("Enter table numbers (e.g., 1 2):", 60)

The robot parses and validates the list of tables.

Delivery to Each Table:

    Navigates to the table

    Waits for Enter (confirm) or any other key (cancel)

    Logs and skips if timed out

Handling Undelivered Food:

    if skipped_tables:
        self.go_to("Kitchen", *KITCHEN)
        self.wait_for_input("Chef: Press Enter to take back food.", None)

    If any tables were skipped or cancelled, the robot returns to the kitchen before ending the task.

🧠 Features at a Glance
Feature	                               Status
Start only after user command 's'	✅
Cancel before reaching kitchen	        ✅
Accept multiple table numbers	        ✅
Waits for confirmation at each table	✅
Timeout and cancellation handling	✅
Returns skipped food to kitchen  	✅
Ends by returning to home	        ✅
Works with Nav2 in Gazebo	        ✅


💡 Design Choices

    Modular & clean: Core logic is broken into functions (go_to, wait_for_input, run_loop) for clarity and reuse.

    Interactive: User can cancel or confirm deliveries in real-time.

    Non-blocking & safe: Navigation waits are blocking, but user prompts have timeouts, preventing lockups.

    Extensible: Can easily add features like voice input, GUI, or dynamic goals later.

    Generic Logic: No hardcoded table flow; handles any input (e.g., 1 3, 2, 3 1).



© 2025 – Arshath
GitHub: github.com/itsarshathm
