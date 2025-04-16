#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import threading
import time
import sys
import select

class InputHandler:
    @staticmethod
    def check_input(timeout=0.1):
        """Check for user input with timeout"""
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.readline().strip() if ready else None

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        
        # Define location coordinates
        self.locations = {
            'HOME': {'x': -2.999, 'y': 2.9999, 'theta': 0.0},
            'KITCHEN': {'x': -1.71381, 'y': -2.694628, 'theta': 0.0},
            'TABLE1': {'x': -0.740918, 'y': -1.886719, 'theta': 0.0},
            'TABLE2': {'x': -0.860984, 'y': 2.21331, 'theta': 0.0},
            'TABLE3': {'x': 1.509219, 'y': 1.423257, 'theta': 0.0}
        }
        
        # Initialize navigation system
        self.nav = None
        self.navigator_ready = False
        self.quaternions = None
        
        # Start navigator setup in background
        threading.Thread(target=self._initialize_navigation, daemon=True).start()
        
        self.get_logger().info('Multi-Order Butler Robot initializing...')
    
    def _initialize_navigation(self):
        """Initialize the navigation system"""
        try:
            self.nav = BasicNavigator()
            
            # Initialize quaternions for orientation
            self.quaternions = quaternion_from_euler(0.0, 0.0, 0.0)
            
            # Set initial pose (home position)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.locations['HOME']['x']
            initial_pose.pose.position.y = self.locations['HOME']['y']
            initial_pose.pose.orientation.x = self.quaternions[0]
            initial_pose.pose.orientation.y = self.quaternions[1]
            initial_pose.pose.orientation.z = self.quaternions[2]
            initial_pose.pose.orientation.w = self.quaternions[3]
            
            self.nav.setInitialPose(initial_pose)
            self.nav.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.get_logger().info('Navigation system is ready!')
        except Exception as e:
            self.get_logger().error(f'Navigation initialization error: {e}')
    
    def wait_for_navigation_ready(self):
        """Wait until navigation system is initialized"""
        spinner_chars = ['|', '/', '-', '\\']
        i = 0
        while not self.navigator_ready:
            sys.stdout.write(f"\rInitializing navigation {spinner_chars[i % len(spinner_chars)]} ")
            sys.stdout.flush()
            i += 1
            time.sleep(0.1)
        sys.stdout.write("\rNavigation system ready!           \n")
        return True
    
    def create_goal_pose(self, location_key):
        """Create a goal pose for the specified location"""
        location = self.locations[location_key]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = location['x']
        goal_pose.pose.position.y = location['y']
        goal_pose.pose.orientation.x = self.quaternions[0]
        goal_pose.pose.orientation.y = self.quaternions[1]
        goal_pose.pose.orientation.z = self.quaternions[2]
        goal_pose.pose.orientation.w = self.quaternions[3]
        
        return goal_pose
    
    def navigate_to(self, location_key, description):
        """Navigate to a specific location"""
        if not self.navigator_ready:
            self.get_logger().error('Navigation system not ready!')
            return TaskResult.FAILED
        
        self.get_logger().info(f'Navigating to {description}...')
        
        # Create and send goal
        goal_pose = self.create_goal_pose(location_key)
        self.nav.goToPose(goal_pose)
        
        # Monitor navigation progress
        last_feedback_time = time.time()
        
        while not self.nav.isTaskComplete():
            # Display feedback periodically
            current_time = time.time()
            if current_time - last_feedback_time >= 1.0:
                last_feedback_time = current_time
                try:
                    feedback = self.nav.getFeedback()
                    if feedback:
                        remaining = feedback.distance_remaining
                        print(f"Distance to {description}: {remaining:.2f}m")
                except Exception:
                    pass
                    
            time.sleep(0.1)
        
        # Get final result
        result = self.nav.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Successfully reached {description}!')
        else:
            self.get_logger().error(f'Failed to reach {description}!')
            
        return result
    
    def wait_at_location(self, seconds, prompt=None):
        """Wait at current location for specified seconds with optional prompt for input"""
        if prompt:
            print(prompt)
        
        start_time = time.time()
        user_response = None
        
        while time.time() - start_time < seconds and user_response is None:
            user_input = InputHandler.check_input(0.1)
            if user_input:
                if user_input.lower() == 'confirm':
                    user_response = 'confirm'
                elif user_input.lower() == 'cancel':
                    user_response = 'cancel'
            
            # Show countdown
            remaining = int(seconds - (time.time() - start_time))
            if remaining % 2 == 0:  # Update every 2 seconds
                print(f"Waiting: {remaining}s remaining for input...", end='\r')
        
        print(" " * 50, end='\r')  # Clear the line
        return user_response
    
    def process_multi_table_order(self, table_nums):
        """Process orders for multiple tables in sequence"""
        if not self.wait_for_navigation_ready():
            return False
            
        # Validate table numbers
        valid_tables = []
        for num in table_nums:
            if 1 <= num <= 3:
                valid_tables.append(num)
        
        if not valid_tables:
            self.get_logger().error('No valid table numbers provided')
            return False
            
        self.get_logger().info(f'Processing orders for tables: {valid_tables}')
        
        # Step 1: Go from home to kitchen
        print("Moving to kitchen to pick up all orders...")
        result = self.navigate_to('KITCHEN', 'Kitchen')
        
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error('Failed to reach kitchen, returning home')
            self.navigate_to('HOME', 'Home')
            return False
        
        # Wait at kitchen and ask for confirmation for all orders
        response = self.wait_at_location(10, "Confirm or cancel orders? (type 'confirm' or 'cancel')")
        
        # If canceled or no response at kitchen, return to home
        if response != 'confirm':
            self.get_logger().info('Orders not confirmed at kitchen. Returning to home.')
            self.navigate_to('HOME', 'Home')
            return False
            
        # Step 2: Deliver to each table in sequence
        for table_num in valid_tables:
            table_key = f'TABLE{table_num}'
            print(f"\nDelivering to Table {table_num}...")
            
            result = self.navigate_to(table_key, f'Table {table_num}')
            if result != TaskResult.SUCCEEDED:
                self.get_logger().error(f'Failed to reach Table {table_num}, skipping')
                continue
                
            # Wait at table for delivery confirmation
            print(f"Delivered to Table {table_num}")
            self.wait_at_location(10)
        
        # Return to home after all deliveries
        self.get_logger().info('All deliveries completed. Returning to home position.')
        self.navigate_to('HOME', 'Home')
        return True
    
    def parse_table_input(self, input_str):
        """Parse table numbers from input string"""
        table_nums = []
        
        # Strip "table" prefix if present
        clean_input = input_str.lower().replace('table', '').strip()
        
        # Process each character or digit
        for char in clean_input:
            if char.isdigit():
                table_num = int(char)
                if 1 <= table_num <= 3:
                    table_nums.append(table_num)
        
        return table_nums
    
    def run(self):
        """Main operation loop"""
        self.wait_for_navigation_ready()
        
        print("\n=== Multi-Order Butler Robot Ready ===")
        print("Enter table numbers in sequence (e.g., '123', 'table 321', '12', etc.)")
        print("Press 'q' to quit\n")
        
        running = True
        while running and rclpy.ok():
            try:
                user_input = input("Enter command: ").strip()
                
                # Check for quit command
                if user_input.lower() == 'q':
                    running = False
                    continue
                    
                # Check for table numbers
                table_nums = self.parse_table_input(user_input)
                if table_nums:
                    print(f"Processing order sequence: {table_nums}")
                    self.process_multi_table_order(table_nums)
                else:
                    print("Invalid command. Enter table numbers (e.g., '123', 'table 231') or 'q' to quit.")
                    
            except KeyboardInterrupt:
                running = False
                
        # Clean shutdown
        if self.nav and self.navigator_ready:
            self.nav.lifecycleShutdown()

def main():
    rclpy.init()
    
    try:
        robot = ButlerRobot()
        robot.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
