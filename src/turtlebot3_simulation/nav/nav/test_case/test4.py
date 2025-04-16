#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import threading
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
        
        # Initialize navigator in a separate thread
        self.navigator = None
        self.navigator_ready = False
        self.is_navigation_active = False
        self.current_destination = None
        self.cancel_requested = False
        
        # Start navigator setup in background
        threading.Thread(target=self._initialize_navigation, daemon=True).start()
        
        self.get_logger().info('Butler Robot starting up...')
        
    def _initialize_navigation(self):
        """Initialize the navigation system"""
        try:
            self.navigator = BasicNavigator()
            
            # Set initial pose (home position)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.locations['HOME']['x']
            initial_pose.pose.position.y = self.locations['HOME']['y']
            initial_pose.pose.orientation.w = 1.0
            
            self.navigator.setInitialPose(initial_pose)
            self.navigator.waitUntilNav2Active()
            
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
        sys.stdout.write("\rNavigation system ready!        \n")
        return True
    
    def create_goal_pose(self, location_key):
        """Create a goal pose for the specified location"""
        location = self.locations[location_key]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = location['x']
        goal_pose.pose.position.y = location['y']
        goal_pose.pose.orientation.w = 1.0
        
        return goal_pose
    
    def navigate_to(self, location_key, description, check_cancel=True):
        """Navigate to a specific location with cancellation checking"""
        if not self.navigator_ready:
            self.get_logger().error('Navigation system not ready!')
            return TaskResult.FAILED
        
        self.get_logger().info(f'Navigating to {description}...')
        self.current_destination = location_key
        self.is_navigation_active = True
        self.cancel_requested = False
        
        # Create and send goal
        goal_pose = self.create_goal_pose(location_key)
        self.navigator.goToPose(goal_pose)
        
        # Monitor navigation progress
        start_time = time.time()
        last_feedback_time = start_time
        
        while not self.navigator.isTaskComplete():
            # Check for cancellation input
            if check_cancel:
                user_input = InputHandler.check_input(0.1)
                if user_input == 'C':
                    self.get_logger().info('Cancellation requested!')
                    self.cancel_requested = True
                    self.navigator.cancelTask()
                    time.sleep(0.5)
                    return TaskResult.CANCELED
            
            # Display feedback periodically
            current_time = time.time()
            if current_time - last_feedback_time >= 1.0:
                last_feedback_time = current_time
                try:
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        remaining = feedback.distance_remaining
                        print(f"Distance to {description}: {remaining:.2f}m")
                except Exception:
                    pass
                    
            time.sleep(0.1)
        
        # Get final result
        result = self.navigator.getResult()
        self.is_navigation_active = False
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Successfully reached {description}!')
        else:
            self.get_logger().error(f'Failed to reach {description}!')
            
        return result
    
    def wait_at_location(self, seconds, prompt=None):
        """Wait at current location for specified seconds with optional prompt"""
        if prompt:
            print(prompt)
        
        start_time = time.time()
        user_responded = False
        
        while time.time() - start_time < seconds:
            user_input = InputHandler.check_input(0.1)
            if user_input:
                if prompt:  # Only process input if there's a prompt
                    if user_input.lower() == 'confirm':
                        return 'confirm'
                    elif user_input.lower() == 'cancel':
                        return 'cancel'
                elif user_input == 'C':
                    return 'cancel'
            
            # Show countdown
            remaining = int(seconds - (time.time() - start_time))
            if remaining % 2 == 0:  # Update every 2 seconds
                print(f"Waiting: {remaining}s remaining...", end='\r')
        
        print(" " * 40, end='\r')  # Clear the line
        return None  # No user input received
    
    def process_table_order(self, table_num):
        """Process an order for the specified table with cancellation logic"""
        if not self.wait_for_navigation_ready():
            return False
            
        table_key = f'TABLE{table_num}'
        if table_key not in self.locations:
            self.get_logger().error(f'Invalid table number: {table_num}')
            return False
            
        self.get_logger().info(f'Processing order for Table {table_num}')
        
        # Step 1: Go from home to kitchen
        print("Moving to kitchen to pick up order...")
        result = self.navigate_to('KITCHEN', 'Kitchen')
        
        # If canceled while going to kitchen, return to home
        if result == TaskResult.CANCELED or self.cancel_requested:
            self.get_logger().info('Order canceled while going to kitchen. Returning to home.')
            self.navigate_to('HOME', 'Home', check_cancel=False)
            return False
        
        # Wait at kitchen and ask for confirmation
        response = self.wait_at_location(10, "Confirm or cancel order? (type 'confirm' or 'cancel')")
        
        # If canceled or no response at kitchen, return to home
        if response == 'cancel' or response is None:
            self.get_logger().info('Order canceled at kitchen. Returning to home.')
            self.navigate_to('HOME', 'Home', check_cancel=False)
            return False
            
        # Step 2: Go from kitchen to table
        print(f"Taking order to Table {table_num}...")
        result = self.navigate_to(table_key, f'Table {table_num}')
        
        # If canceled while going to table, return to kitchen then home
        if result == TaskResult.CANCELED or self.cancel_requested:
            self.get_logger().info('Order canceled while going to table. Returning to kitchen.')
            self.navigate_to('KITCHEN', 'Kitchen', check_cancel=False)
            self.wait_at_location(10)
            self.navigate_to('HOME', 'Home', check_cancel=False)
            return False
            
        # Wait at table and ask for confirmation
        response = self.wait_at_location(10, "Confirm or cancel delivery? (type 'confirm' or 'cancel')")
        
        # Handle response at table
        if response == 'cancel':
            self.get_logger().info('Delivery canceled at table. Returning to kitchen.')
            self.navigate_to('KITCHEN', 'Kitchen', check_cancel=False)
            self.wait_at_location(10)
            self.navigate_to('HOME', 'Home', check_cancel=False)
            return False
        else:
            # Whether confirmed or timed out, return to home
            self.get_logger().info('Returning to home position.')
            self.navigate_to('HOME', 'Home', check_cancel=False)
            return True
            
    def run(self):
        """Main operation loop"""
        self.wait_for_navigation_ready()
        
        print("\n=== Butler Robot Ready ===")
        print("Enter table number (1, 2, or 3) to start delivery process")
        print("Press 'q' to quit\n")
        
        running = True
        while running and rclpy.ok():
            try:
                user_input = input("Enter command: ").strip()
                
                # Check for quit command
                if user_input.lower() == 'q':
                    running = False
                    continue
                    
                # Check for table number
                if user_input in ['1', '2', '3']:
                    table_num = int(user_input)
                    self.process_table_order(table_num)
                else:
                    print("Invalid command. Enter 1, 2, or 3 for table, or 'q' to quit.")
                    
            except KeyboardInterrupt:
                running = False
                
        # Clean shutdown
        if self.navigator and self.navigator_ready:
            self.navigator.lifecycleShutdown()

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
