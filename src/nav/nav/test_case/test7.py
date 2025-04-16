#!/usr/bin/env python3
import threading
import queue
import io
import sys
import time
from enum import Enum

# ROS 2 Integration
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RobotState(Enum):
    IDLE = 0
    GOING_TO_KITCHEN = 1
    AT_KITCHEN = 2
    GOING_TO_TABLE = 3
    AT_TABLE = 4
    RETURNING_HOME = 5
    WAITING = 6

class RestaurantRobot:
    def __init__(self):
        self.input_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.table_sequence = []
        self.state = RobotState.IDLE
        
        threading.Thread(target=self.init_ros, daemon=True).start()
        threading.Thread(target=self.init_robot, daemon=True).start()

    def init_ros(self):
        try:
            rclpy.init()
            print("ROS 2 node initialized successfully.")
        except Exception as e:
            print(f"Error initializing ROS 2: {e}")

    def init_robot(self):
        try:
            self.robot = ROS2Robot(self)
            self.robot_thread = threading.Thread(target=self.robot.run, daemon=True)
            self.robot_thread.start()
            print("Robot Ready")
            
        except Exception as e:
            print(f"Error initializing robot: {e}")

    def add_to_sequence(self, table_num):
        if len(self.table_sequence) >= 3:
            print("Maximum 3 tables in sequence")
            return
            
        if table_num not in self.table_sequence:
            self.table_sequence.append(table_num)
            self.print_sequence()
    
    def print_sequence(self):
        if not self.table_sequence:
            print("Sequence: None")
        else:
            sequence_text = " → ".join([f"T{num}" for num in self.table_sequence])
            print(f"Sequence: {sequence_text}")
    
    def clear_sequence(self):
        self.table_sequence = []
        print("Sequence cleared")
        
    def start_delivery(self):
        if not self.table_sequence:
            print("Empty Sequence - Please add at least one table to the sequence")
            return
        
        if self.state != RobotState.IDLE:
            print("Robot is currently busy. Please wait.")
            return
        
        table_str = ','.join([str(num) for num in self.table_sequence])
        print(f"\nProcessing order sequence: {self.table_sequence}\n")
        self.input_queue.put(f"deliver:{table_str}")
    
    def go_home(self):
        self.input_queue.put("home")
        
    def shutdown(self):
        self.input_queue.put("quit")
        if rclpy.ok():
            rclpy.shutdown()
            
    def process_kitchen_confirmation(self, table_nums):
        # Command line version instead of GUI dialog
        print(f"At kitchen, confirming orders for tables: {table_nums}...")
        print("Enter table numbers to confirm (comma-separated) or 'all'/'none':")
        
        try:
            user_input = input()
            
            if user_input.lower() == 'all':
                confirmed_tables = table_nums
                canceled_tables = []
            elif user_input.lower() == 'none':
                confirmed_tables = []
                canceled_tables = table_nums
            else:
                try:
                    confirmed_tables = [int(t.strip()) for t in user_input.split(',') if t.strip()]
                    confirmed_tables = [t for t in confirmed_tables if t in table_nums]
                    canceled_tables = [t for t in table_nums if t not in confirmed_tables]
                except ValueError:
                    print("Invalid input. Canceling all orders.")
                    confirmed_tables = []
                    canceled_tables = table_nums
            
            if confirmed_tables:
                print(f"Confirmed orders for tables: {confirmed_tables}")
            
            if canceled_tables:
                print(f"Canceled orders for tables: {canceled_tables}")
                
            if not confirmed_tables:
                print("All orders canceled at kitchen. Returning home.")
                
            self.clear_sequence()
            return confirmed_tables, canceled_tables
            
        except Exception as e:
            print(f"Error processing confirmation: {e}")
            print("No response received at kitchen, canceling all orders")
            return [], table_nums

    def process_table_confirmation(self, table_num):
        # Command line version instead of GUI dialog
        print(f"Reached Table {table_num}. Confirming delivery...")
        print(f"Confirm delivery at Table {table_num}? (yes/no):")
        
        try:
            user_input = input().strip().lower()
            
            if user_input in ('yes', 'y'):
                print(f"Delivery confirmed at Table {table_num}.")
                return True
            else:
                print(f"Delivery canceled at Table {table_num}.")
                return False
                
        except Exception as e:
            print(f"Error processing confirmation: {e}")
            print(f"No response received at Table {table_num}, skipping delivery")
            return False

class ROS2Robot(Node):
    def __init__(self, robot_controller):
        super().__init__('restaurant_robot')
        self.controller = robot_controller
        
        # Updated with exact coordinates
        self.locations = {
            'HOME': {'ros_x': -2.999, 'ros_y': 2.9999, 'theta': 0.0},
            'KITCHEN': {'ros_x': -1.71381, 'ros_y': -2.694628, 'theta': 0.0},
            'TABLE1': {'ros_x': -0.740918, 'ros_y': -1.886719, 'theta': 0.0},
            'TABLE2': {'ros_x': -0.860984, 'ros_y': 2.21331, 'theta': 0.0},
            'TABLE3': {'ros_x': 1.509219, 'ros_y': 1.423257, 'theta': 0.0}
        }
        
        self.wait_time_at_table = 10  # Exact 10 seconds wait time
        self.current_position = 'HOME'
        self.running = True
        
        # ROS 2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        print("ROS 2 Robot initialized successfully.")

    def move_to_location(self, loc_name):
        if not self.running:
            return False
            
        print(f"Moving to {loc_name}...")
        
        if loc_name not in self.locations:
            print(f"Unknown location: {loc_name}")
            return False
            
        target_loc = self.locations[loc_name]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position using the exact coordinates
        goal_msg.pose.pose.position.x = target_loc['ros_x']
        goal_msg.pose.pose.position.y = target_loc['ros_y']
        goal_msg.pose.pose.orientation.w = 1.0  # Simple orientation
        
        # Send goal
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print(f"Goal to {loc_name} was rejected")
            return False
            
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        if get_result_future.result().status == 4:  # SUCCEEDED
            print(f"Reached {loc_name} successfully!")
            self.current_position = loc_name
            return True
        else:
            print(f"Failed to reach {loc_name}")
            return False

    def wait_at_location(self, seconds, message=None):
        """
        Wait at current location for exact specified time.
        Fixed to ensure precise timing without overruns.
        """
        if message:
            print(f"{message}")
        
        # Record exact start time
        start_time = time.time()
        end_time = start_time + seconds
        
        # Update displayed countdown more frequently than once per second
        update_interval = 0.1  # Update status 10 times per second
        next_display_time = start_time + update_interval
        last_displayed_seconds = seconds
        
        while time.time() < end_time and self.running:
            current_time = time.time()
            remaining = max(0, end_time - current_time)
            
            # Update display when whole second changes or it's time for an update
            seconds_remaining = int(remaining)
            if (seconds_remaining != last_displayed_seconds or 
                current_time >= next_display_time) and message:
                print(f"Waiting: {seconds_remaining}s remaining...", end='\r')
                last_displayed_seconds = seconds_remaining
                next_display_time = current_time + update_interval
            
            # Sleep in short intervals to remain responsive
            # Only sleep for the shorter of 0.1s or the remaining time
            sleep_time = min(0.1, end_time - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                break
        
        # Clear status line and print completion message
        print(" " * 50, end='\r')
        
        # Calculate actual waited time
        actual_wait = time.time() - start_time
        print(f"Wait completed. (Waited for {actual_wait:.1f} seconds)")

    def go_home(self):
        print("Returning to home position...")
        self.controller.state = RobotState.RETURNING_HOME
        
        success = self.move_to_location('HOME')
        self.controller.state = RobotState.IDLE
        
        if success:
            print("At home position. Ready for new commands.")
        else:
            print("Failed to reach home position.")
            
    def deliver_to_tables(self, table_nums):
        if not table_nums:
            print("No tables in delivery sequence.")
            return
            
        print(f"Starting delivery to tables: {table_nums}")
        
        # First go to kitchen
        self.controller.state = RobotState.GOING_TO_KITCHEN
        kitchen_success = self.move_to_location('KITCHEN')
        
        if not kitchen_success:
            print("Failed to reach kitchen. Aborting delivery.")
            self.controller.state = RobotState.IDLE
            return
            
        self.controller.state = RobotState.AT_KITCHEN
        confirmed_tables, canceled_tables = self.controller.process_kitchen_confirmation(table_nums)
        
        if not confirmed_tables:
            self.go_home()
            return
            
        # Deliver to all confirmed tables first
        for table_num in confirmed_tables:
            if not self.running:
                break
                
            table_loc = f'TABLE{table_num}'
            self.controller.state = RobotState.GOING_TO_TABLE
            print(f"Delivering to Table {table_num}...")
            
            table_success = self.move_to_location(table_loc)
            
            if not table_success:
                print(f"Failed to reach Table {table_num}. Skipping.")
                continue
                
            self.controller.state = RobotState.AT_TABLE
            delivery_confirmed = self.controller.process_table_confirmation(table_num)
            
            if delivery_confirmed:
                # Use the exact wait time from class variable
                self.wait_at_location(self.wait_time_at_table, f"Completing service at Table {table_num}")
            else:
                print(f"Skipping service at Table {table_num}")
        
        # After all deliveries, return to kitchen
        self.controller.state = RobotState.GOING_TO_KITCHEN
        kitchen_success = self.move_to_location('KITCHEN')
        
        if not kitchen_success:
            print("Failed to return to kitchen. Going home.")
        
        # Finally return home
        self.go_home()

    def run(self):
        print("ROS 2 robot running. Ready to accept commands.")
        
        while self.running and rclpy.ok():
            try:
                cmd = self.controller.input_queue.get(timeout=0.1)
                self.controller.input_queue.task_done()
                
                if cmd == "quit":
                    print("Shutting down robot...")
                    self.running = False
                    break
                    
                elif cmd == "home":
                    if self.controller.state == RobotState.IDLE:
                        self.go_home()
                        
                elif cmd.startswith("deliver:"):
                    if self.controller.state == RobotState.IDLE:
                        table_str = cmd.split(":", 1)[1]
                        table_nums = [int(t) for t in table_str.split(',')]
                        self.deliver_to_tables(table_nums)
                        
            except queue.Empty:
                pass
                
            time.sleep(0.1)
            
        print("Robot thread terminated.")

# Main function to run the robot from command line
def main():
    robot = RestaurantRobot()
    
    print("\nRestaurant Robot - Command Line Interface")
    print("Commands:")
    print("  add <table_num>  - Add table to sequence")
    print("  clear           - Clear table sequence")
    print("  start           - Start delivery")
    print("  home            - Go to home position")
    print("  quit            - Exit program")
    
    try:
        while True:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == "quit":
                robot.shutdown()
                break
            elif cmd.startswith("add "):
                try:
                    table_num = int(cmd.split()[1])
                    if 1 <= table_num <= 3:
                        robot.add_to_sequence(table_num)
                    else:
                        print("Invalid table number. Use 1-3.")
                except (ValueError, IndexError):
                    print("Invalid command. Use 'add <table_num>'")
            elif cmd == "clear":
                robot.clear_sequence()
            elif cmd == "start":
                robot.start_delivery()
            elif cmd == "home":
                robot.go_home()
            else:
                print("Unknown command. Try again.")
                
    except KeyboardInterrupt:
        print("\nExiting...")
        robot.shutdown()

if __name__ == "__main__":
    main()
