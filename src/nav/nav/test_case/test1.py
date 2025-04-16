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
import math

class InputHandler:
    @staticmethod
    def check_input(timeout=0.1):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.readline().strip() if ready else None

class RestaurantRobot(Node):
    def __init__(self):
        super().__init__('restaurant_robot')
        
        # Timeout settings
        self.location_wait_time = 10  # Wait 10 seconds at each location
        self.nav_timeout = 120.0
        
        # Define locations with the provided coordinates
        self.locations = {
            'HOME': {"x": -2.999, "y": 2.9999, "theta": 0.0},
            'KITCHEN': {"x": -1.71381, "y": -2.694628, "theta": 0.0},
            'TABLE1': {"x": -0.740918, "y": -1.886719, "theta": 0.0},
            'TABLE2': {"x": -0.860984, "y": 2.21331, "theta": 0.0},
            'TABLE3': {"x": 1.509219, "y": 1.423257, "theta": 0.0}
        }
        
        # Initialize state variables
        self.current_destination = "HOME"
        self.navigator_ready = False
        
        # Start navigator in background
        threading.Thread(target=self._initialize_navigation, daemon=True).start()
        
        self.get_logger().info('Test Case 1 Robot Starting - Automatic Delivery')
        
    def _initialize_navigation(self):
        try:
            self.nav = BasicNavigator()
            
            # Initialize quaternions for orientation
            self.quaternions = [quaternion_from_euler(0.0, 0.0, 0.0)]
                
            # Set initial pose (home)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.locations['HOME']["x"]
            initial_pose.pose.position.y = self.locations['HOME']["y"]
            initial_pose.pose.orientation.x = self.quaternions[0][0]
            initial_pose.pose.orientation.y = self.quaternions[0][1]
            initial_pose.pose.orientation.z = self.quaternions[0][2]
            initial_pose.pose.orientation.w = self.quaternions[0][3]
            
            self.nav.setInitialPose(initial_pose)
            self.nav.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.get_logger().info('Navigation ready! Enter table number (1, 2, or 3):')
        except Exception as e:
            self.get_logger().error(f'Navigation initialization error: {e}')
    
    def _create_goal_pose(self, position):
        """Create a goal pose from position"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = position["x"]
        goal.pose.position.y = position["y"]
        goal.pose.orientation.x = self.quaternions[0][0]
        goal.pose.orientation.y = self.quaternions[0][1]
        goal.pose.orientation.z = self.quaternions[0][2]
        goal.pose.orientation.w = self.quaternions[0][3]
        return goal
    
    def wait_for_navigation_ready(self):
        """Wait until navigation is initialized"""
        if not self.navigator_ready:
            spinner_chars = ['|', '/', '-', '\\']
            i = 0
            while not self.navigator_ready:
                sys.stdout.write(f"\rInitializing navigation {spinner_chars[i % len(spinner_chars)]} ")
                sys.stdout.flush()
                i += 1
                time.sleep(0.1)
            sys.stdout.write("\rNavigation ready!        \n")
        return True
    
    def navigate_to(self, location_key, location_name):
        """Navigate to a location"""
        if not self.wait_for_navigation_ready():
            return TaskResult.FAILED
            
        self.get_logger().info(f'Going to {location_name}...')
            
        # Set destination and start navigation
        self.current_destination = location_name
        goal_pose = self._create_goal_pose(self.locations[location_key])
        
        # Cancel any ongoing navigation
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()
            time.sleep(0.5)
            
        self.nav.goToPose(goal_pose)
        
        # Monitor navigation progress
        last_status_time = time.time()
        start_time = time.time()
        
        print(f"Distance to {location_name}: calculating...")
        
        while not self.nav.isTaskComplete():
            # Check timeout
            if time.time() - start_time > self.nav_timeout:
                self.get_logger().warning(f'Navigation timeout after {self.nav_timeout}s')
                self.nav.cancelTask()
                return TaskResult.FAILED
            
            # Show distance periodically
            current_time = time.time()
            if current_time - last_status_time >= 1.0:
                last_status_time = current_time
                try:
                    feedback = self.nav.getFeedback()
                    if feedback:
                        print(f"Distance to {location_name}: {feedback.distance_remaining:.2f}m")
                except:
                    pass
            
            time.sleep(0.05)
        
        # Get result
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached {location_name}!')
        else:
            self.get_logger().error(f'Failed to reach {location_name}!')
        return result
    
    def handle_order(self, table_num):
        """Handle delivery to a table"""
        if not self.wait_for_navigation_ready():
            return False
        
        table_key = f'TABLE{table_num}'
        
        if table_key not in self.locations:
            self.get_logger().error(f'Invalid table number: {table_num}')
            return False
            
        self.get_logger().info(f'Processing order for Table {table_num}')
        
        # Go to kitchen
        kitchen_result = self.navigate_to('KITCHEN', "Kitchen")
        if kitchen_result != TaskResult.SUCCEEDED:
            self.get_logger().error(f'Failed to reach Kitchen, going home')
            self.navigate_to('HOME', "Home")
            return False
            
        # Wait at kitchen for specified time
        self.get_logger().info(f'Waiting at Kitchen for {self.location_wait_time} seconds...')
        time.sleep(self.location_wait_time)
        
        # Go to table
        table_result = self.navigate_to(table_key, f"Table {table_num}")
        if table_result != TaskResult.SUCCEEDED:
            self.get_logger().error(f'Failed to reach Table {table_num}, going home')
            self.navigate_to('HOME', "Home")
            return False
            
        # Wait at table for specified time
        self.get_logger().info(f'Waiting at Table {table_num} for {self.location_wait_time} seconds...')
        time.sleep(self.location_wait_time)
        
        # Return home
        self.get_logger().info('Delivery complete, returning home')
        home_result = self.navigate_to('HOME', "Home")
        
        return home_result == TaskResult.SUCCEEDED
    
    def process_command(self, command):
        """Process user commands"""
        if not command:
            return True
            
        # Special commands
        if command.lower() == 'q':
            return False
            
        if command.lower() == 'help':
            print("\nCOMMANDS:")
            print("- 1, 2, 3: Order for specific table")
            print("- q: Quit")
            return True
                
        # Table orders
        if command.isdigit():
            try:
                table_num = int(command)
                if 1 <= table_num <= 3:
                    self.handle_order(table_num)
                else:
                    print('Invalid table number. Use 1, 2, or 3')
                return True
            except ValueError:
                print('Invalid input. Enter a table number 1-3')
                return True
                
        print(f'Unknown command: {command}')
        print('Enter a table number (1, 2, or 3) or "q" to quit')
        return True
        
    def run(self):
        """Main robot operation loop"""
        print("\nTEST CASE 1: Automatic delivery without confirmations")
        print("Enter a table number (1, 2, or 3) to start delivery")
        print("Enter 'q' to quit")
        
        running = True
        while running and rclpy.ok():
            try:
                command = input("\nEnter table number: ")
                running = self.process_command(command)
                
            except KeyboardInterrupt:
                running = False
                
        # Clean shutdown
        if self.navigator_ready:
            try:
                self.nav.lifecycleShutdown()
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {e}')

def main():
    try:
        rclpy.init()
        robot = RestaurantRobot()
        robot.run()
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
