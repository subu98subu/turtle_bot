#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
import time
import math
import sys
import os
from enum import Enum
import numpy as np
from functools import partial

# ROS2 dependency handling
try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    try:
        from transforms3d.euler import euler2quat
        
        def quaternion_from_euler(roll, pitch, yaw):
            return euler2quat(roll, pitch, yaw, 'sxyz')
    except ImportError:
        print("WARNING: Neither tf_transformations nor transforms3d is installed.")
        print("Install one of them with: pip install transforms3d")
        
        def quaternion_from_euler(roll, pitch, yaw):
            # A very basic quaternion from euler implementation
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            q = [0, 0, 0, 0]
            q[0] = cy * cp * cr + sy * sp * sr
            q[1] = cy * cp * sr - sy * sp * cr
            q[2] = sy * cp * sr + cy * sp * cr
            q[3] = sy * cp * cr - cy * sp * sr
            return q

class RobotState(Enum):
    IDLE = 0
    GOING_TO_KITCHEN = 1
    AT_KITCHEN = 2
    GOING_TO_TABLE = 3
    AT_TABLE = 4
    RETURNING_HOME = 5
    WAITING = 6

class Location:
    def __init__(self, name, x, y, theta=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        
    def get_position(self):
        return (self.x, self.y)
        
    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class Table(Location):
    def __init__(self, number, x, y, theta=0.0):
        super().__init__(f"TABLE{number}", x, y, theta)
        self.number = number
        self.order_ready = False
        self.order_delivered = False
        self.canceled = False

class RestaurantRobotCLI(Node):
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('restaurant_robot_cli')
        
        # Define parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confirmation_timeout', 15.0),
                ('nav_timeout', 30.0),
                ('simulation_speed', 1.0)
            ]
        )
        
        self.confirmation_timeout = self.get_parameter('confirmation_timeout').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.simulation_speed = self.get_parameter('simulation_speed').value
        
        # Robot state and position
        self.state = RobotState.IDLE
        self.destination = None
        self.current_table = None
        
        # Define locations
        self.home = Location("HOME", 400, 500)
        self.kitchen = Location("KITCHEN", 200, 200)
        self.tables = [
            Table(1, 600, 200, 0.0),
            Table(2, 600, 350, 0.0),
            Table(3, 600, 500, 0.0)
        ]
        
        # Current position
        self.robot_pos = (self.home.x, self.home.y)
        
        # Active orders
        self.active_orders = []
        self.pending_orders = []
        self.completed_orders = []
        self.canceled_tables = []
        
        # ROS2 initialization
        self.navigator = None
        self.navigator_ready = False
        self.ros_thread = None
        
        # Command processing flag
        self.is_running = True
        
        # Start command processing thread
        self.command_thread = threading.Thread(target=self._process_commands, daemon=True)
        self.command_thread.start()
        
        # ROS2 navigation (if available)
        try:
            self._initialize_ros_navigation()
        except Exception as e:
            self.log(f"ROS2 navigation initialization failed: {e}")
            self.log("Running in simulation mode only")
    
    def _initialize_ros_navigation(self):
        """Initialize ROS2 navigation in a separate thread"""
        self.ros_thread = threading.Thread(target=self._setup_navigation, daemon=True)
        self.ros_thread.start()
    
    def _setup_navigation(self):
        """Setup ROS2 navigation stack"""
        try:
            self.navigator = BasicNavigator()
            
            # Set initial pose at home
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0  # Adjusted for ROS2 map
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            
            self.navigator.setInitialPose(initial_pose)
            self.log("Waiting for navigation to become active...")
            self.navigator.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.log("ROS2 navigation is ready!")
        except Exception as e:
            self.log(f"Navigation setup error: {e}")
            self.navigator_ready = False
    
    def _create_goal_pose(self, location):
        """Create a goal pose for navigation"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Convert GUI coordinates to ROS2 map coordinates (simplified)
        # In a real system, you would need proper coordinate transformation
        goal.pose.position.x = (location.x - 400) / 100.0  # Center x=400 is 0,0 in ROS2
        goal.pose.position.y = (500 - location.y) / 100.0  # Center y=500 is 0,0 in ROS2
        
        q = quaternion_from_euler(0.0, 0.0, location.theta)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        return goal
    
    def navigate_to(self, location):
        """Navigate to a location using ROS2 (or simulate if unavailable)"""
        self.destination = location
        
        if self.navigator_ready and self.navigator:
            # Use ROS2 navigation
            goal_pose = self._create_goal_pose(location)
            self.log(f"Navigating to {location.name} via ROS2")
            self.navigator.goToPose(goal_pose)
            
            # Monitor in separate thread
            threading.Thread(target=self._monitor_navigation, args=(location,), daemon=True).start()
        else:
            # Simulate navigation
            self.log(f"Simulating navigation to {location.name}")
            threading.Thread(target=self._simulate_navigation, args=(location,), daemon=True).start()
    
    def _monitor_navigation(self, target_location):
        """Monitor ROS2 navigation progress"""
        start_time = time.time()
        
        while not self.navigator.isTaskComplete():
            # Check for timeout
            if time.time() - start_time > self.nav_timeout:
                self.log(f"Navigation timeout to {target_location.name}")
                self.navigator.cancelTask()
                self.state = RobotState.IDLE
                return
            
            try:
                feedback = self.navigator.getFeedback()
                if feedback:
                    # Update position (approximation)
                    ros_x = feedback.current_pose.pose.position.x
                    ros_y = feedback.current_pose.pose.position.y
                    
                    # Convert ROS coordinates to internal coordinates
                    gui_x = ros_x * 100.0 + 400
                    gui_y = 500 - ros_y * 100.0
                    
                    self.robot_pos = (gui_x, gui_y)
            except Exception:
                pass
            
            time.sleep(0.1)
        
        # Navigation complete
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.log(f"Reached {target_location.name}")
            self.robot_pos = (target_location.x, target_location.y)
            self._handle_arrival(target_location)
        else:
            status = "CANCELED" if result == TaskResult.CANCELED else "FAILED"
            self.log(f"Navigation to {target_location.name} {status}")
            self.state = RobotState.IDLE
    
    def _simulate_navigation(self, target_location):
        """Simulate robot navigation"""
        start_pos = self.robot_pos
        end_pos = (target_location.x, target_location.y)
        
        # Calculate distance and time
        distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        speed = 100.0 * self.simulation_speed  # pixels per second
        duration = distance / speed
        
        start_time = time.time()
        elapsed = 0
        
        while elapsed < duration:
            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / duration)
            
            # Linear interpolation
            new_x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
            new_y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress
            
            self.robot_pos = (new_x, new_y)
            time.sleep(0.02)
        
        # Ensure final position is exact
        self.robot_pos = end_pos
        self._handle_arrival(target_location)
    
    def _handle_arrival(self, location):
        """Handle robot arrival at destination"""
        if location.name == "HOME":
            self.state = RobotState.IDLE
            self.log("Returned to home position")
            
            # If we have pending orders, process the next one
            if self.pending_orders:
                table_num = self.pending_orders.pop(0)
                self.process_order(table_num)
            
        elif location.name == "KITCHEN":
            self.state = RobotState.AT_KITCHEN
            self.log("Arrived at kitchen, waiting for confirmation")
            
            # Set waiting state for CLI input
            self.state = RobotState.WAITING
            self.log("Enter 'confirm' or 'deny' to proceed:")
            
        elif location.name.startswith("TABLE"):
            table = next((t for t in self.tables if t.name == location.name), None)
            if table:
                self.current_table = table
                self.state = RobotState.AT_TABLE
                self.log(f"Arrived at Table {table.number}, waiting for confirmation")
                
                # Set waiting state for CLI input
                self.state = RobotState.WAITING
                self.log("Enter 'confirm' or 'deny' to proceed:")
    
    def process_order(self, table_num):
        """Process a new order for a table"""
        if self.state != RobotState.IDLE:
            self.log(f"Robot busy, queuing order for Table {table_num}")
            self.pending_orders.append(table_num)
            return
        
        # Find the table
        table = next((t for t in self.tables if t.number == table_num), None)
        if not table:
            self.log(f"Invalid table number: {table_num}")
            return
        
        # Reset table state
        table.order_ready = False
        table.order_delivered = False
        table.canceled = False
        
        # Add to active orders
        if table_num not in self.active_orders:
            self.active_orders.append(table_num)
        
        # Go to kitchen first
        self.log(f"Processing order for Table {table_num}")
        self.state = RobotState.GOING_TO_KITCHEN
        self.navigate_to(self.kitchen)
    
    def process_multiple_orders(self, table_nums):
        """Process multiple orders simultaneously"""
        # Queue up all orders
        for table_num in table_nums:
            # Find the table
            table = next((t for t in self.tables if t.number == table_num), None)
            if not table:
                self.log(f"Invalid table number: {table_num}")
                continue
            
            # Reset table state
            table.order_ready = False
            table.order_delivered = False
            table.canceled = False
            
            # Add to active orders
            if table_num not in self.active_orders:
                self.active_orders.append(table_num)
        
        # Start processing if robot is idle
        if self.state == RobotState.IDLE:
            self.log(f"Processing multiple orders for Tables: {table_nums}")
            self.state = RobotState.GOING_TO_KITCHEN
            self.navigate_to(self.kitchen)
        else:
            self.log(f"Robot busy, queuing orders for Tables: {table_nums}")
            self.pending_orders.extend([t for t in table_nums if t not in self.pending_orders])
    
    def handle_kitchen_confirmation(self, confirmed=True):
        """Handle confirmation at kitchen"""
        self.state = RobotState.GOING_TO_TABLE
        
        if not confirmed:
            self.log("Order not ready at kitchen, returning home")
            self.state = RobotState.RETURNING_HOME
            self.navigate_to(self.home)
            return
        
        # If we have multiple active orders, go to each table in sequence
        if len(self.active_orders) > 0:
            # Visit the first table
            table_num = self.active_orders[0]
            table = next((t for t in self.tables if t.number == table_num), None)
            
            if table and not table.canceled:
                self.log(f"Going to Table {table_num}")
                table.order_ready = True
                self.navigate_to(table)
            else:
                # Skip canceled tables
                if table:
                    self.active_orders.remove(table_num)
                    self.log(f"Table {table_num} order canceled")
                
                if self.active_orders:
                    self.handle_kitchen_confirmation(True)  # Process next table
                else:
                    self.log("No active orders, returning home")
                    self.state = RobotState.RETURNING_HOME
                    self.navigate_to(self.home)
    
    def handle_table_confirmation(self, confirmed=True):
        """Handle confirmation at table"""
        if not self.current_table:
            self.log("No current table selected")
            self.state = RobotState.RETURNING_HOME
            self.navigate_to(self.home)
            return
        
        table_num = self.current_table.number
        
        if confirmed:
            self.log(f"Order delivered to Table {table_num}")
            self.current_table.order_delivered = True
            
            # Remove from active orders
            if table_num in self.active_orders:
                self.active_orders.remove(table_num)
            
            # Add to completed orders
            if table_num not in self.completed_orders:
                self.completed_orders.append(table_num)
        else:
            self.log(f"No confirmation at Table {table_num}")
        
        # If we have more active orders, process next table
        if self.active_orders:
            next_table_num = self.active_orders[0]
            next_table = next((t for t in self.tables if t.number == next_table_num), None)
            
            if next_table and not next_table.canceled:
                self.log(f"Going to next Table {next_table_num}")
                self.state = RobotState.GOING_TO_TABLE
                self.navigate_to(next_table)
                return
        
        # If no more tables or all canceled, return home via kitchen
        if self.state != RobotState.RETURNING_HOME:
            self.log("Going back to kitchen before home")
            self.state = RobotState.GOING_TO_KITCHEN
            self.navigate_to(self.kitchen)
    
    def cancel_order(self, table_num):
        """Cancel an order for a specific table"""
        table = next((t for t in self.tables if t.number == table_num), None)
        if not table:
            self.log(f"Invalid table number: {table_num}")
            return
        
        table.canceled = True
        if table_num in self.active_orders:
            self.active_orders.remove(table_num)
        if table_num in self.pending_orders:
            self.pending_orders.remove(table_num)
        
        self.log(f"Order for Table {table_num} canceled")
        
        # If current destination is this table, go back home
        if self.current_table and self.current_table.number == table_num:
            self.log("Currently heading to canceled table, returning to kitchen")
            self.navigate_to(self.kitchen)
    
    def cancel_all_orders(self):
        """Cancel all orders and return home"""
        self.active_orders = []
        self.pending_orders = []
        
        for table in self.tables:
            table.canceled = False
            table.order_ready = False
            table.order_delivered = False
        
        self.log("All orders canceled, returning home")
        self.state = RobotState.RETURNING_HOME
        self.navigate_to(self.home)
    
    def go_home(self):
        """Return directly to home position"""
        self.log("Going home")
        self.state = RobotState.RETURNING_HOME
        self.navigate_to(self.home)
    
    def log(self, message):
        """Print a log message to the console"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        print(log_message)
    
    def status(self):
        """Print current robot status"""
        print("\n=== ROBOT STATUS ===")
        print(f"State: {self.state.name}")
        print(f"Position: ({int(self.robot_pos[0])}, {int(self.robot_pos[1])})")
        if self.destination:
            print(f"Destination: {self.destination.name}")
        print(f"Active orders: {', '.join(f'T{o}' for o in self.active_orders) if self.active_orders else 'None'}")
        print(f"Pending orders: {', '.join(f'T{o}' for o in self.pending_orders) if self.pending_orders else 'None'}")
        print(f"Completed orders: {', '.join(f'T{o}' for o in self.completed_orders) if self.completed_orders else 'None'}")
        print("===================\n")
    
    def _process_commands(self):
        """Process commands from the terminal"""
        self.log("Restaurant Robot CLI started. Type 'help' for available commands.")
        
        while self.is_running:
            try:
                if sys.stdin.isatty():  # Only prompt if running in interactive terminal
                    command = input("Enter command: ")
                else:
                    command = input()
                
                parts = command.strip().lower().split()
                if not parts:
                    continue
                
                cmd = parts[0]
                
                if cmd == "help":
                    print("\nAvailable commands:")
                    print("  order <table_number>     - Process an order for a table")
                    print("  multi <t1,t2,...>        - Process orders for multiple tables")
                    print("  cancel <table_number>    - Cancel an order for a table")
                    print("  cancel_all               - Cancel all orders")
                    print("  home                     - Return to home position")
                    print("  status                   - Show current robot status")
                    print("  confirm                  - Confirm at current location")
                    print("  deny                     - Deny at current location")
                    print("  quit                     - Exit the program")
                    print("  help                     - Show this help message")
                
                elif cmd == "order" and len(parts) > 1:
                    try:
                        table_num = int(parts[1])
                        self.process_order(table_num)
                    except ValueError:
                        self.log("Invalid table number. Use a number between 1-3.")
                
                elif cmd == "multi" and len(parts) > 1:
                    try:
                        if "," in parts[1]:
                            table_nums = [int(t) for t in parts[1].split(",")]
                        else:
                            table_nums = [int(t) for t in parts[1:]]
                        self.process_multiple_orders(table_nums)
                    except ValueError:
                        self.log("Invalid table numbers. Format: multi 1,2,3 or multi 1 2 3")
                
                elif cmd == "cancel" and len(parts) > 1:
                    try:
                        table_num = int(parts[1])
                        self.cancel_order(table_num)
                    except ValueError:
                        self.log("Invalid table number")
                
                elif cmd == "cancel_all":
                    self.cancel_all_orders()
                
                elif cmd == "home":
                    self.go_home()
                
                elif cmd == "status":
                    self.status()
                
                elif cmd == "confirm":
                    if self.state == RobotState.WAITING:
                        if self.destination.name == "KITCHEN":
                            self.handle_kitchen_confirmation(True)
                        elif self.destination.name.startswith("TABLE"):
                            self.handle_table_confirmation(True)
                        else:
                            self.log("No confirmation needed at current location")
                    else:
                        self.log("Robot is not waiting for confirmation")
                
                elif cmd == "deny":
                    if self.state == RobotState.WAITING:
                        if self.destination.name == "KITCHEN":
                            self.handle_kitchen_confirmation(False)
                        elif self.destination.name.startswith("TABLE"):
                            self.handle_table_confirmation(False)
                        else:
                            self.log("No confirmation needed at current location")
                    else:
                        self.log("Robot is not waiting for confirmation")
                
                elif cmd == "quit" or cmd == "exit":
                    self.log("Exiting...")
                    self.is_running = False
                    if rclpy.ok():
                        self.destroy_node()
                    break
                
                else:
                    self.log(f"Unknown command: {command}. Type 'help' for available commands.")
                
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\nExiting...")
                self.is_running = False
                break
            except Exception as e:
                self.log(f"Error processing command: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        robot = RestaurantRobotCLI()
        
        # Main loop
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(robot)
        
        try:
            while rclpy.ok() and robot.is_running:
                executor.spin_once(timeout_sec=0.1)
                time.sleep(0.01)  # Short sleep to avoid CPU overload
        except KeyboardInterrupt:
            print("Keyboard interrupt received, shutting down...")
        finally:
            executor.shutdown()
            robot.destroy_node()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
