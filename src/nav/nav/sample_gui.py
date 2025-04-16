#!/usr/bin/env python3
import pygame
import pygame_gui
import time
import threading
import math
from enum import Enum
import os
import sys
import random

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
    NAVIGATING = 1
    AT_LOCATION = 2
    WAITING_FOR_CONFIRMATION = 3
    RETURNING_HOME = 4

class Location:
    def __init__(self, name, x, y, ros_x, ros_y, ros_theta=0.0):
        self.name = name
        self.x = x  # GUI coordinate
        self.y = y  # GUI coordinate
        self.ros_x = ros_x  # ROS2 coordinate
        self.ros_y = ros_y  # ROS2 coordinate
        self.ros_theta = ros_theta  # ROS2 orientation

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
        self.waiting_for_confirmation = False
        self.confirmation_timer = 0
        
        # Define locations (GUI coords and ROS2 coords)
        self.home = Location("HOME", 400, 150, -2.999, 2.9999)
        self.kitchen = Location("KITCHEN", 200, 450, -1.71381, -2.694628)
        
        # Additional locations can be added here
        self.locations = {
            "HOME": self.home,
            "KITCHEN": self.kitchen,
        }
        
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
        pygame.display.set_caption("Butler Robot Control Interface")
        
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
            "BUTLER ROBOT CONTROL",
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
        
        # Navigation control panel
        self.nav_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 150, 280, 200),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.nav_title = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "NAVIGATION CONTROL",
            self.gui_manager,
            container=self.nav_panel
        )
        
        # Navigation buttons - these will be dynamically added by test cases
        self.nav_buttons = {}
        
        # Confirmation panel
        self.confirm_panel = pygame_gui.elements.UIPanel(
            pygame.Rect(self.control_panel_rect.x + 10, 360, 280, 130),
            starting_layer_height=1,
            manager=self.gui_manager
        )
        
        self.confirm_label = pygame_gui.elements.UILabel(
            pygame.Rect(10, 5, 260, 20),
            "ACTIONS",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        self.confirm_button = pygame_gui.elements.UIButton(
            pygame.Rect(10, 35, 80, 35),
            "Confirm",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        self.cancel_button = pygame_gui.elements.UIButton(
            pygame.Rect(100, 35, 80, 35),
            "Cancel",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        self.reset_button = pygame_gui.elements.UIButton(
            pygame.Rect(190, 35, 80, 35),
            "Reset",
            self.gui_manager,
            container=self.confirm_panel
        )
        
        # Add a button for adding custom locations
        self.add_location_button = pygame_gui.elements.UIButton(
            pygame.Rect(10, 80, 260, 35),
            "Add Custom Location",
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
        
        # Update state
        if location.name == "HOME":
            self.state = RobotState.IDLE
            self.update_status("IDLE")
        else:
            self.state = RobotState.AT_LOCATION
            self.update_status(f"At {location.name} - Need confirmation")
            self.waiting_for_confirmation = True
            self.confirmation_timer = self.confirmation_timeout
            
        # Extension point for test cases to handle specific location arrivals
        self.on_location_arrival(location)
    
    def on_location_arrival(self, location):
        """Extension point for test cases to handle location arrivals"""
        # This method can be overridden by test cases
        pass
    
    def handle_confirmation(self):
        """Handle manual confirmation at current location"""
        if not self.waiting_for_confirmation:
            return
            
        self.waiting_for_confirmation = False
        self.add_log(f"Action at {self.destination.name} confirmed")
        
        # Extension point for test cases to handle confirmations
        if self.on_confirmation():
            return
            
        # Default behavior: return home
        self.state = RobotState.RETURNING_HOME
        self.navigate_to_location(self.home)
    
    def on_confirmation(self):
        """Extension point for test cases to handle confirmations"""
        # Return True if handled, False to use default behavior
        return False
    
    def handle_cancellation(self):
        """Handle cancellation at current location or during navigation"""
        if self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            self.add_log("Operation cancelled!")
            self.state = RobotState.RETURNING_HOME
            self.navigate_to_location(self.home)
                
        elif self.state != RobotState.IDLE:
            self.current_task_canceled = True
            
            # Cancel navigation if active
            if self.nav and not self.nav.isTaskComplete():
                self.nav.cancelTask()
                
            self.add_log("Task canceled")
            
            # Reset state
            self.state = RobotState.IDLE
            self.update_status("IDLE")
    
    def reset_system(self):
        """Reset the robot system to initial state"""
        # Cancel any ongoing tasks
        self.current_task_canceled = True
        if self.nav and not self.nav.isTaskComplete():
            self.nav.cancelTask()
        
        # Reset robot state
        self.state = RobotState.IDLE
        self.update_status("IDLE")
        self.waiting_for_confirmation = False
        
        # Reset robot position
        self.robot_pos = (self.home.x, self.home.y)
        self.path_points = []
        
        # Extension point for test cases to do additional reset
        self.on_reset()
        
        self.add_log("System reset complete")
    
    def on_reset(self):
        """Extension point for test cases to handle system reset"""
        pass
    
    def add_location(self, name, x, y, ros_x, ros_y, ros_theta=0.0):
        """Add a custom location"""
        if name in self.locations:
            self.add_log(f"Location {name} already exists")
            return False
            
        # Create new location
        location = Location(name, x, y, ros_x, ros_y, ros_theta)
        self.locations[name] = location
        
        # Add navigation button
        button_y = 35 + (len(self.nav_buttons) * 40)
        button = pygame_gui.elements.UIButton(
            pygame.Rect(10, button_y, 260, 30),
            f"Go to {name}",
            self.gui_manager,
            container=self.nav_panel,
            object_id=f"nav_to_{name}"
        )
        self.nav_buttons[name] = button
        
        self.add_log(f"Added location: {name}")
        return True
    
    def show_add_location_dialog(self):
        """Show dialog to add a custom location"""
        # This would be implemented with a dialog box in a real application
        # For simplicity, we'll just add a random location
        loc_num = len(self.locations) + 1
        name = f"LOC{loc_num}"
        x = random.randint(100, 600)
        y = random.randint(100, 400)
        ros_x = random.uniform(-3.0, 3.0)
        ros_y = random.uniform(-3.0, 3.0)
        
        self.add_location(name, x, y, ros_x, ros_y)
    
    def draw_location(self, location, location_color=(0, 0, 180)):
        """Draw a location on the map"""
        # Draw location marker
        pygame.draw.circle(self.screen, location_color, (location.x, location.y), 10)
        
        # Draw location label
        font = pygame.font.SysFont(None, 24)
        text = font.render(location.name, True, (0, 0, 0))
        self.screen.blit(text, (location.x - text.get_width() // 2, location.y - 30))
    
    def draw_robot(self):
        """Draw the robot on the map"""
        # Draw path if enabled
        if self.show_path and len(self.path_points) > 1:
            pygame.draw.lines(self.screen, (200, 0, 0), False, self.path_points, 2)
            
        # Draw projected position circle
        pygame.draw.circle(self.screen, (255, 200, 200), self.robot_pos, 20, 1)
        
        # Draw robot with rotation
        rotated_robot = pygame.transform.rotate(self.robot_image, -self.robot_angle)
        robot_rect = rotated_robot.get_rect(center=self.robot_pos)
        self.screen.blit(rotated_robot, robot_rect)
    
    def run(self):
        """Main application loop"""
        try:
            # Initialize basic navigation buttons
            self.add_location("HOME", self.home.x, self.home.y, self.home.ros_x, self.home.ros_y)
            self.add_location("KITCHEN", self.kitchen.x, self.kitchen.y, self.kitchen.ros_x, self.kitchen.ros_y)
            
            while self.running:
                # Handle events
                time_delta = self.clock.tick(60) / 1000.0
                
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    
                    # Process GUI events
                    self.gui_manager.process_events(event)
                    
                    # Handle button events
                    if event.type == pygame_gui.UI_BUTTON_PRESSED:
                        # Navigation buttons
                        for loc_name, button in self.nav_buttons.items():
                            if event.ui_element == button:
                                self.destination = self.locations[loc_name]
                                self.state = RobotState.NAVIGATING
                                self.navigate_to_location(self.destination)
                                break
                        
                        # Control buttons
                        if event.ui_element == self.confirm_button:
                            self.handle_confirmation()
                        elif event.ui_element == self.cancel_button:
                            self.handle_cancellation()
                        elif event.ui_element == self.reset_button:
                            self.reset_system()
                        elif event.ui_element == self.add_location_button:
                            self.show_add_location_dialog()
                
                # Update GUI
                self.gui_manager.update(time_delta)
                
                # Update confirmation timeout if waiting
                if self.waiting_for_confirmation:
                    self.confirmation_timer -= time_delta
                    if self.confirmation_timer <= 0:
                        self.add_log("Confirmation timeout, returning home...")
                        self.waiting_for_confirmation = False
                        self.state = RobotState.RETURNING_HOME
                        self.navigate_to_location(self.home)
                
                # Draw background
                self.screen.fill((230, 230, 230))
                
                # Draw map area
                self.screen.blit(self.map_bg, (0, 0))
                
                # Draw all locations
                for loc in self.locations.values():
                    self.draw_location(loc)
                
                # Draw robot
                self.draw_robot()
                
                # Draw panels
                pygame.draw.rect(self.screen, (240, 240, 240), self.control_panel_rect)
                pygame.draw.rect(self.screen, (220, 220, 220), self.log_panel_rect)
                
                # Extension point for test-specific drawing
                self.on_draw()
                
                # Update the display
                self.gui_manager.draw_ui(self.screen)
                pygame.display.update()
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.add_log(f"Error in main loop: {e}")
        finally:
            # Clean up resources
            pygame.quit()
            if self.nav:
                rclpy.shutdown()
    
    def on_draw(self):
        """Extension point for test cases to add custom drawing"""
        pass

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create and run the application
    app = ButlerRobotGUI()
    
    try:
        # Create a thread for ROS2 spinning
        def spin_thread():
            rclpy.spin(app)
        
        ros_thread = threading.Thread(target=spin_thread, daemon=True)
        ros_thread.start()
        
        # Run pygame main loop
        app.run()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        rclpy.shutdown()

if __name__ == "__main__":
    main()
