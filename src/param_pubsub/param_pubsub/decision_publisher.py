import rclpy                    # import the ROS Client Library for Python (RCLPY)
from rclpy.node import Node     # from RCLPY, import the Node Class used to create ROS 2 nodes
from std_msgs.msg import String # from standard messages, import the String message

#import os
#include_dir = os.path.dirname(os.path.realpath(__file__)) + "/../../../../../../src/include/"
#import sys
#sys.path.append(include_dir)
#from hat_library import *

class DecisionPublisher(Node):   # Create a new class called DecisionPublisher that inherits variables & functions from Node

    def __init__(self):
        super().__init__('decision_publisher')                               # Initialize the Node with the name 'decision_publisher'
        self.publisher_ = self.create_publisher(String, 'robot_decision', 10)  # Create a publisher for String type messages on the topic 'robot_decision'
        
        #self.declare_parameter('my_parameter', 'Hi')                        # Instantiate parameter, set default value to 'Hi'
        #imer_period = 0.5                                                  # Define the timer period in seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)   # Create a timer that calls 'timer_callback' every 0.5 seconds
        self.create_timer(1.0, self.publish_command)  # Create a timer that calls 'timer_callback' every 1 second
        self.publish_command = ["FORWARD", "LEFT", "RIGHT", "STOP"]
        self.current_index = 0

    def publish_command(self):
        #my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        msg = String()                                          # Create a new String message
        msg.data = self.publish_command[self.current_index]    # set msg.data to have the value of my_param
        self.publisher_.publish(msg)                            # Publish the message to the topic
        self.get_logger().info(f'Published command: {msg.data}')   # Log the published message for debugging
        self.current_index = (self.current_index + 1) % len(self.publish_command)

def main(args=None):
    print ("Beginning to talk...")          # Print a starting message
    rclpy.init(args=args)                   # Initialize the ROS 2 Python client library

    decision_publisher = DecisionPublisher()  # Create an instance of the DecisionPublisher class

    try:
        rclpy.spin(decision_publisher)       # Keep the node active and processing callbacks until interrupted

    except KeyboardInterrupt:   # Handle a keyboard interrupt (Ctrl+C)
        print("\n")             # Print a newline for better format
        print("Stopping...")    # Print a stopping message
 
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        decision_publisher.destroy_node()
        if rclpy.ok():                      # Check if the rclpy library is still running
            rclpy.shutdown()                # Shut down the ROS 2 client library, cleanly terminating the node



if __name__ == '__main__':
    main()                  # Call the main function to execute the code when the script is run