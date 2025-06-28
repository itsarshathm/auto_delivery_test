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

