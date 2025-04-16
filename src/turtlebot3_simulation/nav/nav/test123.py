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
import queue
from enum import Enum

class RobotStatus(Enum):
    IDLE = 0
    NAVIGATING = 1
    AT_LOCATION = 2
    WAITING_CONFIRMATION = 3
    RETURNING = 4

class OrderStatus(Enum):
    PENDING = 0
    IN_KITCHEN = 1
    DELIVERING = 2
    DELIVERED = 3
    FAILED = 4
    CANCELED = 5

class InputHandler:
    @staticmethod
    def check_input(timeout=0.1):
        """Check for user input with timeout"""
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.readline().strip() if ready else None

class OrderTask:
    def __init__(self, table_num, order_id):
        self.table_num = table_num
        self.order_id = order_id
        self.status = OrderStatus.PENDING
        self.start_time = time.time()
        self.completion_time = None
        self.at_kitchen_time = None
        self.at_table_time = None
        self.robot_id = None
        self.canceled = False
        
    def calculate_total_time(self):
        if self.completion_time:
            return self.completion_time - self.start_time
        return time.time() - self.start_time
        
    def __str__(self):
        return f"Order #{self.order_id} - Table {self.table_num} - {self.status.name}"

class DeliveryRobot:
    def __init__(self, robot_id, node, locations, quaternions):
        self.robot_id = robot_id
        self.node = node
        self.locations = locations
        self.quaternions = quaternions
        
        self.status = RobotStatus.IDLE
        self.current_order = None
        self.nav = None
        self.navigator_ready = False
        self.current_location = "HOME"
        self.destination = None
        
        # Create a thread for this robot's operations
        self.thread = threading.Thread(target=self._initialize_and_run, daemon=True)
        self.thread.start()
        
        self.node.get_logger().info(f"Robot {robot_id} initialized")
        
    def _initialize_and_run(self):
        """Initialize navigation and run the robot's operation loop"""
        try:
            self.nav = BasicNavigator()
            
            # Set initial pose (home)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.node.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.locations['HOME']["x"]
            initial_pose.pose.position.y = self.locations['HOME']["y"]
            initial_pose.pose.orientation.x = self.quaternions[0][0]
            initial_pose.pose.orientation.y = self.quaternions[0][1]
            initial_pose.pose.orientation.z = self.quaternions[0][2]
            initial_pose.pose.orientation.w = self.quaternions[0][3]
            
            self.nav.setInitialPose(initial_pose)
            self.nav.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.node.get_logger().info(f"Robot {self.robot_id} navigation ready")
            
        except Exception as e:
            self.node.get_logger().error(f"Robot {self.robot_id} navigation initialization error: {e}")
            return
            
    def _create_goal_pose(self, location_key):
        """Create a goal pose from location key"""
        if location_key not in self.locations:
            self.node.get_logger().error(f"Unknown location: {location_key}")
            return None
            
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = self.locations[location_key]["x"]
        goal.pose.position.y = self.locations[location_key]["y"]
        goal.pose.orientation.x = self.quaternions[0][0]
        goal.pose.orientation.y = self.quaternions[0][1]
        goal.pose.orientation.z = self.quaternions[0][2]
        goal.pose.orientation.w = self.quaternions[0][3]
        return goal
        
    def navigate_to(self, location_key, description):
        """Navigate to a location"""
        if not self.navigator_ready:
            self.node.get_logger().error(f"Robot {self.robot_id} navigation not ready")
            return TaskResult.FAILED
            
        # Set destination and status
        self.destination = location_key
        self.status = RobotStatus.NAVIGATING
        
        # Create goal pose and start navigation
        goal_pose = self._create_goal_pose(location_key)
        if goal_pose is None:
            return TaskResult.FAILED
            
        # Cancel any ongoing navigation
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()
            time.sleep(0.5)
            
        # Begin navigation
        self.node.get_logger().info(f"Robot {self.robot_id} navigating to {description}")
        self.nav.goToPose(goal_pose)
        
        # Monitor navigation progress
        last_status_time = time.time()
        start_time = time.time()
        nav_timeout = 120.0  # 2 minutes timeout
        
        while not self.nav.isTaskComplete():
            # Check for cancellation
            if self.current_order and self.current_order.canceled:
                self.node.get_logger().info(f"Robot {self.robot_id} canceling navigation to {description}")
                self.nav.cancelTask()
                return TaskResult.CANCELED
                
            # Check timeout
            if time.time() - start_time > nav_timeout:
                self.node.get_logger().warning(f"Robot {self.robot_id} navigation timeout after {nav_timeout}s")
                self.nav.cancelTask()
                return TaskResult.FAILED
            
            # Show distance periodically
            current_time = time.time()
            if current_time - last_status_time >= 2.0:
                last_status_time = current_time
                try:
                    feedback = self.nav.getFeedback()
                    if feedback:
                        print(f"Robot {self.robot_id} distance to {description}: {feedback.distance_remaining:.2f}m")
                except:
                    pass
            
            time.sleep(0.1)
        
        # Get result
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.node.get_logger().info(f"Robot {self.robot_id} reached {description}")
            self.current_location = location_key
            self.status = RobotStatus.AT_LOCATION
        else:
            self.node.get_logger().error(f"Robot {self.robot_id} failed to reach {description}")
            
        return result
        
    def wait_for_confirmation(self, location_name, timeout=15):
        """Wait for confirmation with timeout"""
        self.status = RobotStatus.WAITING_CONFIRMATION
        
        print(f"Robot {self.robot_id} at {location_name}. Enter 'r{self.robot_id}confirm' to confirm or 'r{self.robot_id}cancel' to cancel.")
        start_time = time.time()
        
        # Wait for timeout or until order is confirmed/canceled
        while time.time() - start_time < timeout:
            if self.current_order and self.current_order.canceled:
                return False
                
            # Sleep to prevent CPU hogging
            time.sleep(0.1)
            
        # If we reach here, confirmation timed out
        self.node.get_logger().warning(f"Robot {self.robot_id} confirmation timeout at {location_name}")
        return False
        
    def process_order(self, order):
        """Process the assigned order"""
        if not self.navigator_ready:
            self.node.get_logger().error(f"Robot {self.robot_id} navigation not ready")
            order.status = OrderStatus.FAILED
            return False
            
        # Assign the order to this robot
        self.current_order = order
        order.robot_id = self.robot_id
        order.status = OrderStatus.IN_KITCHEN
        
        # Go to kitchen first
        self.node.get_logger().info(f"Robot {self.robot_id} going to kitchen for order #{order.order_id}")
        kitchen_result = self.navigate_to('KITCHEN', "Kitchen")
        
        # Check if order was canceled during navigation
        if order.canceled or kitchen_result != TaskResult.SUCCEEDED:
            self.handle_order_cancellation("navigation to kitchen failed")
            return False
            
        # Record kitchen arrival time
        order.at_kitchen_time = time.time()
        
        # Wait for confirmation at kitchen (simulated food prep time)
        kitchen_wait_time = 10  # seconds
        print(f"Robot {self.robot_id} waiting at kitchen for {kitchen_wait_time} seconds...")
        
        # For automated test, just wait fixed time instead of requiring confirmation
        time.sleep(kitchen_wait_time)
        
        # Check if order was canceled during kitchen wait
        if order.canceled:
            self.handle_order_cancellation("canceled during kitchen wait")
            return False
            
        # Update order status and continue to table
        order.status = OrderStatus.DELIVERING
        table_key = f'TABLE{order.table_num}'
        
        # Navigate to the table
        self.node.get_logger().info(f"Robot {self.robot_id} delivering order #{order.order_id} to Table {order.table_num}")
        table_result = self.navigate_to(table_key, f"Table {order.table_num}")
        
        # Check if order was canceled during table navigation
        if order.canceled or table_result != TaskResult.SUCCEEDED:
            self.handle_order_cancellation("navigation to table failed")
            return False
            
        # Record table arrival time
        order.at_table_time = time.time()
        
        # Wait for confirmation at table (simulated delivery time)
        table_wait_time = 10  # seconds
        print(f"Robot {self.robot_id} waiting at Table {order.table_num} for {table_wait_time} seconds...")
        
        # For automated test, just wait fixed time instead of requiring confirmation
        time.sleep(table_wait_time)
        
        # Check if order was canceled during table wait
        if order.canceled:
            self.handle_order_cancellation("canceled during table wait")
            return False
            
        # Update order status to delivered
        order.status = OrderStatus.DELIVERED
        order.completion_time = time.time()
        
        # Return to home
        self.status = RobotStatus.RETURNING
        self.node.get_logger().info(f"Robot {self.robot_id} delivered order #{order.order_id}, returning home")
        home_result = self.navigate_to('HOME', "Home")
        
        # Clear current order and update status
        self.current_order = None
        self.status = RobotStatus.IDLE
        
        return True
        
    def handle_order_cancellation(self, reason):
        """Handle cancellation or failure of current order"""
        order = self.current_order
        if not order:
            return
            
        self.node.get_logger().info(f"Robot {self.robot_id} order #{order.order_id} {reason}")
        order.status = OrderStatus.CANCELED
        
        # Return to home
        self.status = RobotStatus.RETURNING
        self.navigate_to('HOME', "Home")
        self.status = RobotStatus.IDLE
        self.current_order = None
        
    def is_available(self):
        """Check if robot is available for a new order"""
        return self.status == RobotStatus.IDLE and self.navigator_ready and self.current_order is None

class SimultaneousRestaurantRobot(Node):
    def __init__(self):
        super().__init__('simultaneous_restaurant_robot')
        
        # Define locations with the provided coordinates
        self.locations = {
            'HOME': {"x": -2.999, "y": 2.9999, "theta": 0.0},
            'KITCHEN': {"x": -1.71381, "y": -2.694628, "theta": 0.0},
            'TABLE1': {"x": -0.740918, "y": -1.886719, "theta": 0.0},
            'TABLE2': {"x": -0.860984, "y": 2.21331, "theta": 0.0},
            'TABLE3': {"x": 1.509219, "y": 1.423257, "theta": 0.0}
        }
        
        # Initialize quaternions for orientation
        self.quaternions = [quaternion_from_euler(0.0, 0.0, 0.0)]
        
        # Create order queue
        self.order_queue = queue.Queue()
        self.order_history = []
        self.next_order_id = 1
        
        # Create delivery robots (fleet)
        self.robots = {}
        self.num_robots = 3  # We'll create 3 robots
        
        for i in range(1, self.num_robots + 1):
            robot = DeliveryRobot(i, self, self.locations, self.quaternions)
            self.robots[i] = robot
        
        # Start order processing thread
        self.processing_thread = threading.Thread(target=self._process_orders, daemon=True)
        self.processing_thread.start()
        
        # Start status display thread
        self.status_thread = threading.Thread(target=self._display_status, daemon=True)
        self.status_thread.start()
        
        self.get_logger().info('Test Case 3 - Simultaneous Restaurant Robot Starting')
        
    def _display_status(self):
        """Thread to periodically display system status"""
        while True:
            try:
                # Clear screen
                print("\033[H\033[J", end="")
                
                # Print header
                print("=== SIMULTANEOUS RESTAURANT ROBOT SYSTEM ===")
                print(f"Active Orders: {self.order_queue.qsize()}")
                print("Completed/Failed Orders:", len([o for o in self.order_history if o.status in [OrderStatus.DELIVERED, OrderStatus.FAILED, OrderStatus.CANCELED]]))
                print("\n--- ROBOT STATUS ---")
                
                # Show robot status
                for robot_id, robot in self.robots.items():
                    status_text = robot.status.name
                    order_text = f"Order #{robot.current_order.order_id} (Table {robot.current_order.table_num})" if robot.current_order else "None"
                    location = robot.current_location
                    
                    print(f"Robot {robot_id}: {status_text} | Current Order: {order_text} | Location: {location}")
                
                # Show active orders
                if not self.order_queue.empty():
                    print("\n--- PENDING ORDERS ---")
                    # This is a bit tricky since Queue doesn't support peeking at all elements
                    # For a real implementation, you might want to use a different data structure
                    temp_queue = queue.Queue()
                    while not self.order_queue.empty():
                        order = self.order_queue.get()
                        print(f"Order #{order.order_id} - Table {order.table_num} - {order.status.name}")
                        temp_queue.put(order)
                    
                    # Restore the queue
                    while not temp_queue.empty():
                        self.order_queue.put(temp_queue.get())
                
                # Show recent completed orders
                recent_completed = [o for o in self.order_history[-5:] if o.status in [OrderStatus.DELIVERED, OrderStatus.FAILED, OrderStatus.CANCELED]]
                if recent_completed:
                    print("\n--- RECENT COMPLETED ORDERS ---")
                    for order in recent_completed:
                        duration = order.calculate_total_time()
                        print(f"Order #{order.order_id} - Table {order.table_num} - {order.status.name} - Duration: {duration:.1f}s")
                
                # Show commands
                print("\n--- COMMANDS ---")
                print("1-3: Create order for table")
                print("c<order_id>: Cancel specific order")
                print("r<robot_id>confirm: Manually confirm for specific robot")
                print("r<robot_id>cancel: Manually cancel for specific robot")
                print("s: Show detailed system status")
                print("q: Quit")
                
                time.sleep(2)  # Update every 2 seconds
                
            except Exception as e:
                self.get_logger().error(f"Status display error: {e}")
                time.sleep(5)  # Wait before trying again
    
    def _process_orders(self):
        """Thread to process orders from the queue and assign to available robots"""
        while True:
            try:
                # Check if we have any available robots
                available_robots = [r for r in self.robots.values() if r.is_available()]
                
                if available_robots and not self.order_queue.empty():
                    # Get the next order from the queue
                    order = self.order_queue.get()
                    
                    # Assign to first available robot
                    robot = available_robots[0]
                    self.get_logger().info(f"Assigning order #{order.order_id} to Robot {robot.robot_id}")
                    
                    # Start processing the order in the robot's thread
                    threading.Thread(target=robot.process_order, args=(order,), daemon=True).start()
                    
                time.sleep(0.5)  # Check every half second
                
            except Exception as e:
                self.get_logger().error(f"Order processing error: {e}")
                time.sleep(5)  # Wait before trying again
    
    def create_order(self, table_num):
        """Create a new order for the specified table"""
        if table_num < 1 or table_num > 3:
            self.get_logger().error(f"Invalid table number: {table_num}")
            return None
            
        # Create new order
        order = OrderTask(table_num, self.next_order_id)
        self.next_order_id += 1
        
        # Add to queue and history
        self.order_queue.put(order)
        self.order_history.append(order)
        
        self.get_logger().info(f"Created order #{order.order_id} for Table {table_num}")
        return order
    
    def cancel_order(self, order_id):
        """Cancel an order by ID"""
        # Find the order in queue or being processed
        found = False
        
        # Check orders in queue
        temp_queue = queue.Queue()
        while not self.order_queue.empty():
            order = self.order_queue.get()
            if order.order_id == order_id:
                order.status = OrderStatus.CANCELED
                order.canceled = True
                found = True
                self.get_logger().info(f"Canceled queued order #{order_id}")
            temp_queue.put(order)
        
        # Restore queue
        while not temp_queue.empty():
            self.order_queue.put(temp_queue.get())
            
        # Check orders being processed by robots
        for robot in self.robots.values():
            if robot.current_order and robot.current_order.order_id == order_id:
                robot.current_order.canceled = True
                found = True
                self.get_logger().info(f"Marked active order #{order_id} for cancellation")
                
        if not found:
            self.get_logger().warning(f"Order #{order_id} not found")
            
        return found
    
    def show_system_status(self):
        """Display detailed system status"""
        print("\n=== DETAILED SYSTEM STATUS ===")
        
        # Show robot information
        print("\nROBOTS:")
        for robot_id, robot in self.robots.items():
            print(f"Robot {robot_id}:")
            print(f"  Status: {robot.status.name}")
            print(f"  Location: {robot.current_location}")
            print(f"  Navigation Ready: {robot.navigator_ready}")
            if robot.current_order:
                print(f"  Current Order: #{robot.current_order.order_id} for Table {robot.current_order.table_num}")
                print(f"  Order Status: {robot.current_order.status.name}")
            else:
                print("  Current Order: None")
            print()
            
        # Show all orders
        print("ORDERS:")
        for order in self.order_history:
            duration = order.calculate_total_time()
            robot_text = f"Robot {order.robot_id}" if order.robot_id else "Unassigned"
            print(f"Order #{order.order_id} - Table {order.table_num} - {order.status.name} - {robot_text} - Duration: {duration:.1f}s")
        
        print("\nPress Enter to continue...")
        
    def confirm_robot_action(self, robot_id):
        """Manually confirm action for specific robot"""
        if robot_id not in self.robots:
            self.get_logger().warning(f"Invalid robot ID: {robot_id}")
            return False
            
        robot = self.robots[robot_id]
        if robot.status != RobotStatus.WAITING_CONFIRMATION:
            self.get_logger().warning(f"Robot {robot_id} is not waiting for confirmation")
            return False
            
        self.get_logger().info(f"Manually confirmed action for Robot {robot_id}")
        # In a real implementation, this would signal the robot to continue
        return True
        
    def cancel_robot_action(self, robot_id):
        """Manually cancel action for specific robot"""
        if robot_id not in self.robots:
            self.get_logger().warning(f"Invalid robot ID: {robot_id}")
            return False
            
        robot = self.robots[robot_id]
        if not robot.current_order:
            self.get_logger().warning(f"Robot {robot_id} has no current order")
            return False
            
        self.get_logger().info(f"Manually canceled action for Robot {robot_id}")
        robot.current_order.canceled = True
        return True
    
    def process_command(self, command):
        """Process user commands"""
        if not command:
            return True
            
        # Check for quit command
        if command.lower() == 'q':
            return False
            
        # Check for status display command
        if command.lower() == 's':
            self.show_system_status()
            return True
            
        # Check for table order
        if command.isdigit():
            try:
                table_num = int(command)
                if 1 <= table_num <= 3:
                    self.create_order(table_num)
                else:
                    print('Invalid table number. Use 1, 2, or 3')
                return True
            except ValueError:
                print('Invalid input')
                return True
                
        # Check for order cancellation
        if command.startswith('c') and command[1:].isdigit():
            try:
                order_id = int(command[1:])
                self.cancel_order(order_id)
                return True
            except ValueError:
                print('Invalid order ID')
                return True
                
        # Check for robot confirmation
        if command.startswith('r') and 'confirm' in command:
            try:
                robot_id = int(command[1:].split('confirm')[0])
                self.confirm_robot_action(robot_id)
                return True
            except ValueError:
                print('Invalid robot confirmation command')
                return True
                
        # Check for robot cancellation
        if command.startswith('r') and 'cancel' in command:
            try:
                robot_id = int(command[1:].split('cancel')[0])
                self.cancel_robot_action(robot_id)
                return True
            except ValueError:
                print('Invalid robot cancellation command')
                return True
                
        print(f'Unknown command: {command}')
        return True
        
    def run(self):
        """Main robot operation loop"""
        print("\nTEST CASE 3: Simultaneous Restaurant Orders")
        print("Enter a table number (1, 2, or 3) to create an order")
        print("Enter 'c<order_id>' to cancel a specific order")
        print("Enter 'r<robot_id>confirm' or 'r<robot_id>cancel' for robot control")
        print("Enter 's' for detailed status")
        print("Enter 'q' to quit")
        
        running = True
        while running and rclpy.ok():
            try:
                command = input("\nEnter command: ")
                running = self.process_command(command)
                
            except KeyboardInterrupt:
                running = False
                
        # Clean shutdown
        print("Shutting down robots...")
        for robot in self.robots.values():
            if robot.nav:
                try:
                    robot.nav.lifecycleShutdown()
                except Exception as e:
                    self.get_logger().error(f"Error shutting down robot {robot.robot_id}: {e}")

def main():
    try:
        rclpy.init()
        robot = SimultaneousRestaurantRobot()
        robot.run()
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
