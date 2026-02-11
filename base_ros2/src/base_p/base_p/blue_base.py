import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from pynput.keyboard import Listener
import tkinter as tk
import threading
import queue
import subprocess
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
import cv2

class Blue_Base(Node):
    def __init__(self):
        super().__init__('blue_base')
        self.sent_stop = False
        # Specify the device path for the Xbox controller (use your actual device path)
        joystick_device = '/dev/input/controller'  # This name was made with udev rules
        self.get_logger().info(f"Starting joy_node with device: {joystick_device}")

        # Start joy_node with specified device using subprocess
        subprocess.Popen(['ros2', 
                          'run', 
                          'joy', 
                          'joy_node', 
                          '--dev', 
                          joystick_device,
                          '--ros-args',
                          '-p','autorepeat:=0.0'])

        # Subscribe to joystick data
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        # Publisher for commands
        self.cmd_pub = self.create_publisher(
            String,
            'drive_command',
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.message_ID = String()
        self.message_ID.data = "Boop"
        self.base_timer_publisher = self.create_publisher(String, "timer_sent", 10)
        self.base_timer_subscriber = self.create_subscription(String, 'timer_received', self.timer_received, 10)
        self.my_timer =  self.create_timer(1, self.timer_callback)
        # self.last_ping_recieved = 0

        self.trigger_debounce = .3

        self.bridge = CvBridge()
        # GUI for the controller

        # Create a Tkinter window
        self.root = tk.Tk()
        self.root.title("Controller Commander")
        self.root.geometry("600x600")

        # Label to display the current command
        self.label = tk.Label(self.root, text="Hello World", font=('Arial', 12))
        self.label.pack(pady=20)
        self.photo_label = tk.Label(self.root)
        self.photo_label.pack()
        # Queue for inter-thread communication (used to pass commands to the GUI)
        self.command_queue = queue.Queue()

        # Run Tkinter GUI event loop in the main thread
        # self.root.after(100, self.update_gui)  # Start periodic GUI updates
        self.update_gui()
    
    def joy_callback(self, msg):
        # Map controller inputs to command codes based on Xbox controller
        # This is the section of code that I need to edit using the pygame module


        #4 front buttons
        if msg.buttons[0] == 1:  # 1 button pressed
            self.send_command('TURN_LEFT')
            self.command_queue.put("TURN LEFT")
            self.sent_stop = False
        elif msg.buttons[1] == 1:  # 2 button pressed
            self.send_command('GO_BACKWARD')
            self.command_queue.put("GO BACKWARD")
            self.sent_stop = False
        elif msg.buttons[2] == 1:  # 3 button pressed
            self.send_command('TURN_RIGHT')
            self.command_queue.put("TURN RIGHT")
            self.sent_stop = False
        elif msg.buttons[3] == 1:  # 4 button pressed
            self.send_command('GO_FORWARD')
            self.command_queue.put("GO FORWARD")
            self.sent_stop = False

        elif msg.buttons[4] == 1:
            self.get_logger().info('Top Left Bumper')
            self.send_command('TOP_LEFT_BUMPER')
            self.command_queue.put("TOP LEFT BUMPER")
            self.sent_stop = False
        elif msg.buttons[5] == 1:
            self.get_logger().info('Top Right Bumper')
            self.send_command('TOP_RIGHT_BUMPER')
            self.command_queue.put("TOP RIGHT BUMPER")
            self.sent_stop = False
        elif msg.buttons[6] == 1:
            self.get_logger().info('Bottom Left Bumper')
            self.send_command('BOTTOM_LEFT_BUMPER')
            self.command_queue.put("BOTTOM LEFT BUMPER")
            self.sent_stop = False
        elif msg.buttons[7] == 1:
            self.get_logger().info('Bottom Right Bumper')
            self.send_command('BOTTOM_RIGHT_BUMPER')
            self.command_queue.put("BOTTOM RIGHT BUMPER")
            self.sent_stop = False
        elif msg.buttons[8] == 1:
            self.get_logger().info('Button 9')
            self.send_command('BUTTON_9')
            self.command_queue.put("BUTTON 9")
            self.sent_stop = False
        elif msg.buttons[9] == 1:
            self.get_logger().info('Button 10')
            self.send_command('BUTTON_10')
            self.command_queue.put("BUTTON 10")
            self.sent_stop = False
        


        # # Left Joystick
        elif msg.axes[4] > self.trigger_debounce:  
            self.send_command('LEFT_DPAD')
            self.command_queue.put("LEFT DPAD")
            self.sent_stop = False
        elif msg.axes[4] < -1*self.trigger_debounce:  
            self.send_command('RIGHT_DPAD')
            self.command_queue.put("RIGHT DPAD")
            self.sent_stop = False
        elif msg.axes[5] > self.trigger_debounce: 
            self.send_command('UP_DPAD')
            self.command_queue.put("UP DPAD")
            self.sent_stop = False
        elif msg.axes[5] < -1*self.trigger_debounce: 
            self.send_command('DOWN_DPAD')
            self.command_queue.put("DOWN DPAD")
            self.sent_stop = False

        # DPAD
        elif msg.axes[0] > self.trigger_debounce:  
            self.send_command('LEFT_JOY')
            self.command_queue.put("LEFT JOY")
            self.sent_stop = False
        elif msg.axes[0] < -1*self.trigger_debounce:  
            self.send_command('RIGHT_JOY')
            self.command_queue.put("RIGHT JOY")
            self.sent_stop = False
        elif msg.axes[1] > self.trigger_debounce:  
            self.send_command('UP_JOY')
            self.command_queue.put("UP JOY")
            self.sent_stop = False
        elif msg.axes[1] < -1*self.trigger_debounce:
            self.send_command('DOWN_JOY')
            self.command_queue.put("DOWN JOY")
            self.sent_stop = False

        #Right Joystick
        elif msg.axes[2] > self.trigger_debounce:  
            self.send_command('LEFT_JOY2')
            self.command_queue.put("LEFT JOY2")
            self.sent_stop = False
        elif msg.axes[2] < -1*self.trigger_debounce:  
            self.send_command('RIGHT_JOY2')
            self.command_queue.put("RIGHT JOY2")
            self.sent_stop = False
        elif msg.axes[3] > self.trigger_debounce: 
            self.send_command('UP_JOY2')
            self.command_queue.put("UP JOY2")
            self.sent_stop = False
        elif msg.axes[3] < -1*self.trigger_debounce: 
            self.send_command('DOWN_JOY2')
            self.command_queue.put("DOWN JOY2")
            self.sent_stop = False


        else:
            if self.sent_stop is False:
                self.send_command('STOP')
                self.command_queue.put("STOP")
                self.sent_stop = True
    
    def timer_callback(self):
        self.base_timer_publisher.publish(self.message_ID)

    def timer_received(self, msg):
        command_msg = msg.data
        self.get_logger().info(f'Received: {command_msg}')

        

    def send_command(self, command_str):
        command = String()
        command.data = command_str  # Instruction for Pi 2
        self.cmd_pub.publish(command)
        self.get_logger().info(f'Sent: {command_str}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #These are a bunch of converter functions that take the ROS image and conver it into something usable in the GUI
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(cv_image)
        tk_image = ImageTk.PhotoImage(image = pil_image)
        self.photo_label.configure(image=tk_image)
        self.photo_label.image = tk_image


    def update_gui(self):
        # Update the label in the Tkinter GUI with the latest command from the queue
       try:
           command = self.command_queue.get_nowait()  # Get the latest command from the queue
           self.label.config(text=f"Last Command: {command}")
       except queue.Empty:
           pass

        # Continue running the Tkinter event loop
       self.root.after(10, self.update_gui)  # Call this method again after 100ms

    def run_ros_spin(self):
        # Spin ROS 2 in a separate thread to prevent blocking the GUI
        rclpy.spin(self)

def main():
    rclpy.init()
    node = Blue_Base()

    # rclpy.spin(node)
    # node.destroy_node()
    # Run ROS 2 spin in a separate thread
    ros_thread = threading.Thread(target=node.run_ros_spin)
    ros_thread.start()

    # Run Tkinter GUI event loop in the main thread
    node.root.mainloop()

    # Ensure ROS 2 shuts down gracefully
    ros_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()