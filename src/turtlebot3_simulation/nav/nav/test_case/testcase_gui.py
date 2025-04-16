#!/usr/bin/env python3
import pygame
import pygame_gui
import time
import threading
import math
from enum import Enum
import os
import sys

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

# Set environment variable for display
os.environ['SDL_VIDEODRIVER'] = 'x11'

class RobotState(Enum):
    IDLE = 0
    GOING_TO_KITCHEN = 1
    AT_KITCHEN = 2
    GOING_TO_TABLE = 3
    AT_TABLE = 4
    RETURNING_HOME = 5
    CHARGING = 6  # New state for test case

class Location:
    def __init__(self, name, x, y, ros_x, ros_y, ros_theta=0.0):
        self.name = name
        self.x = x  # GUI coordinate
        self.y = y  # GUI coordinate
        self.ros_x = ros_x  # ROS2 coordinate
        self.ros_y = ros_y  # ROS2 coordinate
        self.ros_theta = ros_theta  # ROS2 orientation

class Table(Location):
    def __init__(self, number, x, y, ros_x, ros_y, ros_theta=0.0):
        super().__init__(f"TABLE{number}", x, y, ros_x, ros_y, ros_theta)
        self.number = number
        self.order_delivered = False
        self.urgent = False  # New property for urgency test case

class ButlerRobotGUI(Node):
    def __init__(self):
        super().__init__('butler_robot_gui')
        
        # Initialize ROS2 navigation
        self.nav = None
        self.navigator_ready = False
        self.quaternions = [quaternion_from_euler(0.0, 0.0, 0.0)]
        self.nav_thread = None
        self.nav_timeout = 120.0  # Navigation timeout in seconds
        self.confirmation_timeout = 15.0  # Confirmation timeout
        
        # Robot state
        self.state = RobotState.IDLE
        self.destination = None
        self.current_table = None
        self.waiting_for_confirmation = False
        self.confirmation_timer = 0
        
        # Define locations (GUI coords and ROS2 coords)
        self.home = Location("HOME", 400, 150, -2.999, 2.9999)
        self.kitchen = Location("KITCHEN", 200, 450, -1.71381, -2.694628)
        self.charging_station = Location("CHARGING", 150, 150, -3.5, 3.5)  # New charging station
        self.tables = [
            Table(1, 600, 350, -0.740918, -1.886719),
            Table(2, 550, 200, -0.860984, 2.21331),
            Table(3, 350, 300, 1.509219, 1.423257)
        ]
        
        # Battery level simulation (for test case)
        self.battery_level = 100
        self.battery_drain_rate = 0.05  # Battery drain per second
        self.low_battery_threshold = 30
        self.charging_rate = 0.2  # Battery charge per second
        self.last_battery_update = time.time()
        
        # Path visualization
        self.path_points = []
        self.show_path = True
        
        # Current position (GUI coordinates)
        self.robot_pos = (self.home.x, self.home.y)
        self.robot_angle = 0
        
        # Task tracking
        self.current_task_canceled = False
        
        # Log messages
        self.log_messages = []
        
        # Initialize pygame and GUI
        pygame.init()
        self.screen_size = (1000, 700)
        self.screen = pygame.display.set_mode(self.screen_size)
        pygame.display.set_caption("Butler Robot Control Interface - Battery Management Test Case")
        
        # Load assets
        self.load_assets()
        
        # Setup GUI manager
        self.gui_manager = pygame_gui.UIManager(self.screen_size)
        
        self.setup_gui_elements()
        
        # Clock for controlling frame rate
        self.clock = pygame.time.Clock()
        
        # Start the main loop
        self.running = True
        
        # Start ROS2 navigation initialization in background
        self.nav_init_thread = threading.Thread(target=self._initialize_navigation, daemon=True)
        self.nav_init_thread.start()
        
        # Add initial log message
        self.add_log("System initializing...")
        
    def load_assets(self):
        """Load images and initialize visual assets"""
        # Create robot image
        self.robot_image = pygame.Surface((30, 30), pygame.SRCALPHA)
        pygame.draw.circle(self.robot_image, (255, 50, 50), (15, 15), 15)
        pygame.draw.polygon(self.robot_image, (255, 255, 255), [(15, 0), (25, 15), (15, 10)])
        
        # Create a map background
        self.map_bg = pygame.Surface((700, 500))
        self.map_bg.fill((240, 245, 250))
        
        # Draw grid lines
        grid_color = (220, 225, 230)
        grid_spacing = 50
        for x in range(0, 700, grid_spacing):
            pygame.draw.line(self.map_bg, grid_color, (x, 0), (x, 500), 1)
        for y in range(0, 500, grid_spacing):
            pygame.draw.line(self.map_bg, grid_color, (0, y), (700, y), 1)
            
    def setup_gui_elements(self):
        """Set up all GUI elements"""
        # Create panel areas
        self.map_rect = pygame.Rect(0, 0, 700, 500)
        self.control_panel_rect = pygame.Rect(700, 0, 300, 500)
        self.log_panel_rect = pygame.Rect(0, 500, 1000, 200)
        
        # Create title
        self.title_label = pygame_gui.elements.UILabel(
            pygame.Rect(self.control_panel_rect.x + 20, 10, 260, 30),
            "BATTERY MANAGEMENT TEST CASE",
            self.gui_manager
        )
        
        # Status display
        self.status_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 50, 280, 90),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.status_title = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "STATUS",
            self.gui_manager,
            container=self.status_panel
        )
        
        self.status_label = pygame_gui.elements.UILabel(
            pygame.Rect(10, 30, 260, 25),
            "IDLE",
            self.gui_manager,
            container=self.status_panel
        )
        
        self.navigation_status = pygame_gui.elements.UILabel(
            pygame.Rect(10, 55, 260, 25),
            "Navigation: Initializing...",
            self.gui_manager,
            container=self.status_panel
        )
        
        # Battery display panel
        self.battery_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 150, 280, 90),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.battery_title = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "BATTERY STATUS",
            self.gui_manager,
            container=self.battery_panel
        )
        
        self.battery_label = pygame_gui.elements.UILabel(
            pygame.Rect(10, 30, 260, 25),
            f"Battery: {self.battery_level}%",
            self.gui_manager,
            container=self.battery_panel
        )
        
        self.charge_button = pygame_gui.elements.UIButton(
            pygame.Rect(10, 55, 260, 25),
            "Go to Charging Station",
            self.gui_manager,
            container=self.battery_panel
        )
        
        # Table actions panel
        self.table_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 250, 280, 120),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.table_selection_label = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "TABLE ACTIONS",
            self.gui_manager,
            container=self.table_panel
        )
        
        # Table buttons
        button_width = 80
        button_height = 40
        spacing = 5
        
        self.table_buttons = []
        for i in range(3):
            button = pygame_gui.elements.UIButton(
                pygame.Rect(10 + i*(button_width + spacing), 35, button_width, button_height),
                f"Table {i+1}",
                self.gui_manager,
                container=self.table_panel,
                object_id=f"table_{i+1}_button"
            )
            self.table_buttons.append(button)
        
        # Mark urgent button
        self.mark_urgent_button = pygame_gui.elements.UIButton(
            pygame.Rect(10, 80, 260, 30),
            "Mark Selected Table as Urgent",
            self.gui_manager,
            container=self.table_panel
        )
        
        # Confirmation panel
        self.confirm_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 380, 280, 100),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.confirm_label = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "CONFIRMATION",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        self.confirm_button = pygame_gui.elements.UIButton(
            pygame.Rect(10, 35, 125, 55),
            "Confirm",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        self.cancel_button = pygame_gui.elements.UIButton(
            pygame.Rect(145, 35, 125, 55),
            "Cancel",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        # Log window
        self.log_title = pygame_gui.elements.UILabel(
            pygame.Rect(10, self.log_panel_rect.y + 5, 300, 20),
            "SYSTEM LOG",
            self.gui_manager
        )
        
        self.log_textbox = pygame_gui.elements.UITextBox(
            "",
            pygame.Rect(10, self.log_panel_rect.y + 30, 
                       self.log_panel_rect.width - 20, self.log_panel_rect.height - 40),
            self.gui_manager
        )
    
    def add_log(self, message):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.log_messages.append(log_entry)
        
        # Keep only the last 10 messages
        if len(self.log_messages) > 10:
            self.log_messages = self.log_messages[-10:]
        
        # Update the log textbox
        self.log_textbox.html_text = "<br>".join(self.log_messages)
        self.log_textbox.rebuild()
        
        # Also log to ROS
        if hasattr(self, 'get_logger'):
            self.get_logger().info(message)
    
    def update_status(self, status_text):
        """Update the status label"""
        self.status_label.set_text(status_text)
    
    def update_battery_status(self):
        """Update battery status based on robot state and time"""
        current_time = time.time()
        elapsed = current_time - self.last_battery_update
        self.last_battery_update = current_time
        
        if self.state == RobotState.CHARGING:
            # Charging - increase battery level
            self.battery_level = min(100, self.battery_level + self.charging_rate * elapsed)
        else:
            # Discharging - decrease battery level
            drain_multiplier = 1.0
            if self.state != RobotState.IDLE:
                drain_multiplier = 2.0  # Drain faster when moving
            
            self.battery_level = max(0, self.battery_level - (self.battery_drain_rate * drain_multiplier * elapsed))
            
            # Check for low battery
            if self.battery_level < self.low_battery_threshold and self.state != RobotState.GOING_TO_KITCHEN and self.state != RobotState.AT_KITCHEN:
                self.handle_low_battery()
        
        # Update battery label
        battery_text = f"Battery: {self.battery_level:.1f}%"
        if self.state == RobotState.CHARGING:
            battery_text += " (Charging)"
        elif self.battery_level < self.low_battery_threshold:
            battery_text += " (LOW)"
        
        self.battery_label.set_text(battery_text)
    
    def handle_low_battery(self):
        """Handle low battery condition"""
        if self.state != RobotState.CHARGING and self.state != RobotState.IDLE:
            self.add_log("WARNING: Battery level low! Need to charge soon.")
            
            # If battery is critically low, go to charging station
            if self.battery_level < 10:
                self.add_log("CRITICAL: Battery level critical! Going to charging station.")
                # Cancel current task
                self.handle_cancellation()
                # Go to charging station
                self.go_to_charging_station()
    
    def go_to_charging_station(self):
        """Navigate to charging station"""
        if self.state == RobotState.CHARGING:
            self.add_log("Already at charging station")
            return
            
        self.add_log("Going to charging station...")
        self.state = RobotState.RETURNING_HOME  # Use returning home state for navigation
        self.navigate_to_location(self.charging_station)
    
    def _initialize_navigation(self):
        """Initialize ROS2 navigation"""
        try:
            self.nav = BasicNavigator()
            
            # Set initial pose (home)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.home.ros_x
            initial_pose.pose.position.y = self.home.ros_y
            initial_pose.pose.orientation.x = self.quaternions[0][0]
            initial_pose.pose.orientation.y = self.quaternions[0][1]
            initial_pose.pose.orientation.z = self.quaternions[0][2]
            initial_pose.pose.orientation.w = self.quaternions[0][3]
            
            self.nav.setInitialPose(initial_pose)
            self.nav.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.add_log('Navigation system ready!')
            self.navigation_status.set_text("Navigation: Ready")
            
        except Exception as e:
            self.add_log(f'Navigation initialization error: {e}')
            self.navigation_status.set_text("Navigation: Error")
    
    def _create_goal_pose(self, location):
        """Create a goal pose from a location"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = location.ros_x
        goal.pose.position.y = location.ros_y
        goal.pose.orientation.x = self.quaternions[0][0]
        goal.pose.orientation.y = self.quaternions[0][1]
        goal.pose.orientation.z = self.quaternions[0][2]
        goal.pose.orientation.w = self.quaternions[0][3]
        return goal
    
    def navigate_to_location(self, location):
        """Navigate to a location using ROS2"""
        if not self.navigator_ready:
            self.add_log("Navigation system not ready!")
            return False
        
        # Check battery level before navigation
        if location.name != "CHARGING" and self.battery_level < 10:
            self.add_log("Battery too low for navigation! Going to charging station first.")
            self.go_to_charging_station()
            return False
        
        self.add_log(f"Navigating to {location.name}...")
        
        # ROS2 navigation in a separate thread
        def nav_thread_func():
            try:
                # Clear task cancellation flag
                self.current_task_canceled = False
                
                # Create goal pose
                goal_pose = self._create_goal_pose(location)
                
                # Cancel any ongoing navigation
                if not self.nav.isTaskComplete():
                    self.nav.cancelTask()
                    time.sleep(0.5)
                
                # Start navigation
                self.nav.goToPose(goal_pose)
                
                # Monitor navigation progress
                start_time = time.time()
                last_status_time = start_time
                
                # Simulate path points for visualization
                start_pos = self.robot_pos
                end_pos = (location.x, location.y)
                distance = ((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)**0.5
                num_points = max(10, int(distance / 30))
                
                # Clear previous path and add initial point
                self.path_points = [start_pos]
                
                # Generate path preview points
                for i in range(1, num_points + 1):
                    t = i / num_points
                    point = (
                        start_pos[0] + (end_pos[0] - start_pos[0]) * t, 
                        start_pos[1] + (end_pos[1] - start_pos[1]) * t
                    )
                    self.path_points.append(point)
                
                # Calculate angle for robot rotation
                dx = end_pos[0] - start_pos[0]
                dy = end_pos[1] - start_pos[1]
                self.robot_angle = math.degrees(math.atan2(dy, dx))
                
                while not self.nav.isTaskComplete() and not self.current_task_canceled:
                    # Check timeout
                    current_time = time.time()
                    if current_time - start_time > self.nav_timeout:
                        self.add_log(f'Navigation timeout after {self.nav_timeout}s')
                        self.nav.cancelTask()
                        return False
                    
                    # Update GUI position and status periodically
                    if current_time - last_status_time >= 0.5:
                        last_status_time = current_time
                        try:
                            feedback = self.nav.getFeedback()
                            if feedback:
                                # Update remaining distance in GUI
                                self.update_status(f"To {location.name}: {feedback.distance_remaining:.1f}m")
                                
                                # Update GUI position based on progress
                                progress = 1.0 - min(1.0, feedback.distance_remaining / distance)
                                self.robot_pos = (
                                    start_pos[0] + (end_pos[0] - start_pos[0]) * progress,
                                    start_pos[1] + (end_pos[1] - start_pos[1]) * progress
                                )
                        except Exception:
                            pass
                    
                    time.sleep(0.1)
                
                # Check if task was canceled
                if self.current_task_canceled:
                    self.add_log(f"Navigation to {location.name} canceled")
                    return False
                
                # Get result
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.robot_pos = (location.x, location.y)
                    self.handle_successful_navigation(location)
                    return True
                else:
                    self.add_log(f"Navigation to {location.name} failed with result: {result}")
                    return False
                    
            except Exception as e:
                self.add_log(f"Navigation error: {e}")
                return False
        
        # Start navigation in a thread
        self.nav_thread = threading.Thread(target=nav_thread_func, daemon=True)
        self.nav_thread.start()
        
        return True
    
    def handle_successful_navigation(self, location):
        """Handle successful navigation to a location"""
        self.add_log(f"Arrived at {location.name}")
        
        if location.name == "KITCHEN" and self.state == RobotState.GOING_TO_KITCHEN:
            self.state = RobotState.AT_KITCHEN
            self.update_status("At Kitchen - Need confirmation!")
            self.waiting_for_confirmation = True
            self.confirmation_timer = self.confirmation_timeout
            
        elif location.name.startswith("TABLE") and self.state == RobotState.GOING_TO_TABLE:
            self.state = RobotState.AT_TABLE
            self.update_status(f"At Table {self.current_table.number} - Need confirmation!")
            self.waiting_for_confirmation = True
            self.confirmation_timer = self.confirmation_timeout
            
        elif location.name == "HOME":
            self.state = RobotState.IDLE
            self.update_status("IDLE")
            
        elif location.name == "CHARGING":
            self.state = RobotState.CHARGING
            self.update_status("CHARGING")
            self.add_log("Charging battery...")
    
    def handle_confirmation(self):
        """Handle manual confirmation at current location"""
        self.waiting_for_confirmation = False
        
        if self.state == RobotState.AT_KITCHEN:
            # Check if we should go to an urgent table first
            urgent_table = self.get_next_urgent_table()
            if urgent_table:
                self.add_log(f"Urgent delivery: proceeding to Table {urgent_table.number}...")
                self.current_table = urgent_table
                self.state = RobotState.GOING_TO_TABLE
                self.navigate_to_location(urgent_table)
            else:
                # Standard non-urgent delivery
                self.add_log(f"Proceeding to Table {self.current_table.number}...")
                self.state = RobotState.GOING_TO_TABLE
                self.navigate_to_location(self.current_table)
            
        elif self.state == RobotState.AT_TABLE:
            self.add_log(f"Delivered to Table {self.current_table.number}!")
            
            # Mark table as delivered and no longer urgent
            self.current_table.order_delivered = True
            self.current_table.urgent = False
            
            # Check battery level
            if self.battery_level <= self.low_battery_threshold:
                self.add_log("Low battery detected, going to charging station...")
                self.go_to_charging_station()
            else:
                # Return home
                self.add_log("Returning home...")
                self.state = RobotState.RETURNING_HOME
                self.navigate_to_location(self.home)
    
    def handle_cancellation(self):
        """Handle cancellation at current location or during navigation"""
        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            
            if self.state == RobotState.AT_KITCHEN:
                self.add_log("Kitchen operation cancelled!")
                self.state = RobotState.RETURNING_HOME
                self.navigate_to_location(self.home)
                
            elif self.state == RobotState.AT_TABLE:
                self.add_log(f"Table {self.current_table.number} delivery cancelled!")
                self.state = RobotState.RETURNING_HOME
                self.navigate_to_location(self.home)
                
        elif self.state != RobotState.IDLE and self.state != RobotState.CHARGING:
            self.current_task_canceled = True
            
            # Cancel navigation if active
            if self.nav and not self.nav.isTaskComplete():
                self.nav.cancelTask()
                
            self.add_log("Task canceled")
            
            # Reset state
            if self.state != RobotState.CHARGING:
                self.state = RobotState.IDLE
                self.update_status("IDLE")
    
    def start_table_delivery(self, table_num):
        """Start delivery to a specific table"""
        if not self.navigator_ready:
            self.add_log("Navigation system not ready!")
            return
            
        if self.state != RobotState.IDLE and self.state != RobotState.CHARGING:
            self.add_log("Robot is busy!")
            return
            
        if self.battery_level < 10:
            self.add_log("Battery too low for delivery! Going to charging station.")
            self.go_to_charging_station()
            return
        
        # Find the table
        table = self.tables[table_num - 1]
        
        # Set as current table
        self.current_table = table
        
        # Go to kitchen first
        self.add_log(f"Starting delivery for Table {table_num}")
        self.state = RobotState.GOING_TO_KITCHEN
        self.navigate_to_location(self.kitchen)
    
    def mark_table_urgent(self):
        """Mark a table as urgent"""
        # Find which table button was selected
        for i, button in enumerate(self.table_buttons):
            if button.text.endswith("✓"):
                table = self.tables[i]
                table.urgent = not table.urgent
                
                if table.urgent:
                    self.add_log(f"Table {table.number} marked as URGENT")
                    button.set_text(f"Table {i+1} ⚠")
                else:
                    self.add_log(f"Table {table.number} no longer urgent")
                    button.set_text(f"Table {i+1}")
                return
        
        self.add_log("No table selected!")
    
    def toggle_table_selection(self, table_num):
        """Toggle selection state for a table"""
        # Reset all buttons first
        for i, button in enumerate(self.table_buttons):
            table = self.tables[i]
            if i + 1 == table_num:
                # Toggle selected state
                if button.text.endswith("✓") or button.text.endswith("⚠"):
                    button.set_text(f"Table {i+1}")
                else:
                    if table.urgent:
                        button.set_text(f"Table {i+1} ⚠")
                    else:
                        button.set_text(f"Table {i+1} ✓")
            else:
                # Reset other buttons
                if table.urgent:
                    button.set_text(f"Table {i+1} ⚠")
                else:
                    button.set_text(f"Table {i+1}")
    
    def get_next_urgent_table(self):
        """Get the next urgent table for delivery"""
        for table in self.tables:
            if table.urgent and not table.order_delivered:
                return table
        return None
    
    def run(self):
        """Main application loop"""
        while self.running:
            time_delta = self.clock.tick(60) / 1000.0
            
            # Update battery status
            self.update_battery_status()
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                
                if event.type == pygame.USEREVENT:
                    if event.user_type == pygame_gui.UI_BUTTON_PRESSED:
                        # Table buttons
                        for i, button in enumerate(self.table_buttons):
                            if event.ui_element == button:
                                self.toggle_table_selection(i + 1)
                                
                        # Table delivery
                        for i, button in enumerate(self.table_buttons):
                            if event.ui_element == button and button.text.endswith("✓"):
                                self.start_table_delivery(i + 1)
                        
                        # Mark urgent button
                        if event.ui_element == self.mark_urgent_button:
                            self.mark_table_urgent()
                        
                        # Charging station button
                        if event.ui_element == self.charge_button:
                            self.go_to_charging_station()
                        
                        # Confirmation buttons
                        elif event.ui_element == self.confirm_button:
                            if self.waiting_for_confirmation:
                                self.handle_confirmation()
                        elif event.ui_element == self.cancel_button:
                            self.handle_cancellation()
                
                # Pass events to GUI manager
                self.gui_manager.process_events(event)
            
            # Process ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0)
            
            # Update confirmation timer if waiting
            if self.waiting_for_confirmation:
                self.confirmation_timer -= time_delta
                if self.confirmation_timer <= 0:
                    self.add_log("Confirmation timed out! Cancelling operation...")
                    self.handle_cancellation()
            
            # Update GUI
            self.gui_manager.update(time_delta)
            
            # Draw the application
            self.draw()
            
            # Update the display
            pygame.display.update()
        
        # Clean shutdown
        if self.navigator_ready and self.nav:
            try:
                self.nav.lifecycleShutdown()
            except Exception:
                pass
    
    def draw(self):
        """Draw the application"""
        # Fill the background
        self.screen.fill((240, 240, 240))
        
        # Draw map area background
        self.screen.blit(self.map_bg, self.map_rect.topleft)
        
        # Draw locations
	# Draw locations
        # Draw the home location
        pygame.draw.circle(self.screen, (100, 100, 255), (self.home.x, self.home.y), 15)
        self.screen.blit(pygame.font.SysFont(None, 24).render("HOME", True, (0, 0, 0)), 
                         (self.home.x - 30, self.home.y - 30))
        
        # Draw the kitchen location
        pygame.draw.rect(self.screen, (100, 255, 100), 
                         (self.kitchen.x - 20, self.kitchen.y - 20, 40, 40))
        self.screen.blit(pygame.font.SysFont(None, 24).render("KITCHEN", True, (0, 0, 0)), 
                         (self.kitchen.x - 35, self.kitchen.y - 40))
        
        # Draw the charging station
        pygame.draw.polygon(self.screen, (255, 200, 0), 
                           [(self.charging_station.x - 15, self.charging_station.y + 15),
                            (self.charging_station.x, self.charging_station.y - 15),
                            (self.charging_station.x + 15, self.charging_station.y + 15)])
        self.screen.blit(pygame.font.SysFont(None, 20).render("CHARGING", True, (0, 0, 0)), 
                         (self.charging_station.x - 40, self.charging_station.y - 40))
        
        # Draw tables
        for table in self.tables:
            # Base color depends on urgent status
            color = (255, 100, 100) if table.urgent else (200, 200, 200)
            
            # Draw table shape
            pygame.draw.rect(self.screen, color, 
                             (table.x - 25, table.y - 25, 50, 50))
            
            # Draw table label with delivery status
            label_text = f"Table {table.number}"
            if table.order_delivered:
                label_text += " ✓"
            label = pygame.font.SysFont(None, 20).render(label_text, True, (0, 0, 0))
            self.screen.blit(label, (table.x - 25, table.y - 45))
        
        # Draw path if enabled
        if self.show_path and len(self.path_points) > 1:
            pygame.draw.lines(self.screen, (0, 0, 255), False, self.path_points, 2)
        
        # Draw the robot
        rotated_robot = pygame.transform.rotate(self.robot_image, -self.robot_angle)
        robot_rect = rotated_robot.get_rect(center=(self.robot_pos[0], self.robot_pos[1]))
        self.screen.blit(rotated_robot, robot_rect.topleft)
        
        # Draw battery indicator next to robot
        battery_width = 30
        battery_height = 10
        battery_x = self.robot_pos[0] - battery_width // 2
        battery_y = self.robot_pos[1] + 20
        
        # Battery outline
        pygame.draw.rect(self.screen, (0, 0, 0), 
                         (battery_x, battery_y, battery_width, battery_height), 1)
        
        # Battery fill based on level
        fill_width = max(0, int((battery_width - 2) * self.battery_level / 100))
        
        # Battery color based on level
        if self.battery_level > 50:
            fill_color = (0, 255, 0)  # Green
        elif self.battery_level > 20:
            fill_color = (255, 255, 0)  # Yellow
        else:
            fill_color = (255, 0, 0)  # Red
            
        pygame.draw.rect(self.screen, fill_color, 
                         (battery_x + 1, battery_y + 1, fill_width, battery_height - 2))
        
        # Draw control panel background
        pygame.draw.rect(self.screen, (200, 200, 210), self.control_panel_rect)
        
        # Draw log panel background
        pygame.draw.rect(self.screen, (220, 220, 230), self.log_panel_rect)
        
        # Draw GUI elements
        self.gui_manager.draw_ui(self.screen)

def main():
    """Main function to run the application"""
    # Initialize ROS2
    rclpy.init(args=sys.argv)
    
    # Create the application
    app = ButlerRobotGUI()
    
    try:
        # Run the application
        app.run()
    except KeyboardInterrupt:
        # Handle keyboard interrupt
        pass
    except Exception as e:
        # Handle other exceptions
        print(f"Error: {e}")
    finally:
        # Clean up
        app.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
