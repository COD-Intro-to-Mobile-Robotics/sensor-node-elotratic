import rclpy                    # import the ROS Client Library for Python (RCLPY)
from rclpy.node import Node     # from RCLPY, import the Node Class used to create ROS 2 nodes
from std_msgs.msg import Int32 # from standard messages, import the Int32 message


class ListenerNode(Node):  # Create a new class called ListenerNode that inherits variables and functions from Node

    def __init__(self):
        super().__init__('listener_node')          # Initialize the Node with the name 'listener_node'
        self.subscription = self.create_subscription(   # Create a subscription to receive messages
            Int32,                                     # The message type to subscribe to Int32
            'ir_sensor_data',
            ##'param_topic',  #maybe revise                              # The name of the topic to subscribe to
            self.listener_callback,                     # Callback function to handle incoming messages
            10)                                         # Queue size for incoming messages
        self.subscription                               # Prevent an 'unused variable' warning by referencing the subscription

    def listener_callback(self, msg):                       # Callback method to process received messages
        self.get_logger().info('I heard: "%s"' % msg.data)  # Log the received message


def main(args=None):
    print ("Beginning to listen...")            # Print a starting message
    rclpy.init(args=args)                       # Initialize the ROS 2 Python client library

    listener_node = ListenerNode()    # Create an instance of the ListenerNode class

    try:
        rclpy.spin(listener_node)          # Keep the node active and process incoming messages until interrupted

    except KeyboardInterrupt:   # Handle a keyboard interrupt (Ctrl+C)
        print("\n")             # Print a newline for better format
        print("Stopping...")    # Print a stopping message
 
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        listener_node.destroy_node()
        if rclpy.ok():                      # Check if the rclpy library is still running
            rclpy.shutdown()                # Shut down the ROS 2 client library, cleanly terminating the node


if __name__ == '__main__':
    main()                  # Call the main function to execute the code when the script is run