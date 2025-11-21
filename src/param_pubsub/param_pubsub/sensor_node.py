import rclpy                    # import the ROS Client Library for Python (RCLPY)
from rclpy.node import Node     # from RCLPY, import the Node Class used to create ROS 2 nodes
from std_msgs.msg import Int16 # from standard messages, import the Int16 message
import RPi.GPIO as GPIO  # Import the Raspberry Pi GPIO library


class SensorNode(Node):   # Create a new class called SensorNode that inherits variables & functions from Node

    def __init__(self):
        super().__init__('sensor_node')                               # Initialize the Node with the name 'sensor_node'
       
        self.declare_parameter('sensor_position', 'left')                        # Instantiate parameter, set default value to 0
        self.declare_parameter('publish_rate', 1.0)  #Hz                       # Instantiate parameter, set default value to 0
        
        GPIO.setmode(GPIO.BCM)  # Set the GPIO mode to BCM
        self.pin_map = {
            'left': 17,
            'right': 27,
            'front': 22
        }
        sensor_position = self.get_parameter('sensor_position').get_parameter_value().string_value.lower()
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        if sensor_position in self.pin_map:
            self.sensor_pin = self.pin_map[sensor_position]  # Default to left sensor if invalid position
            GPIO.setup(self.sensor_pin, GPIO.IN)  # Set the GPIO pin as input
            self.config_valid = True
        else:
            self.get_logger().error(f"Invalid sensor position: {sensor_position}")
            self.config_valid = False

        if rate <= 0:
            self.get_logger().error(f"Invalid publish rate: {rate}, defaulting to 1 Hz")
            rate = 1.0  # Default to 1 Hz if invalid rate

            self.publisher_ = self.create_publisher(Int16, 'ir_sensor_data', 10)  # Create a publisher for Int16 type messages on the topic 'my_topic'                                             # Define the timer period in seconds
            self.timer = self.create_timer(1.0 / rate, self.timer_callback)   # Create a timer that calls 'timer_callback' every 0.5 seconds

    def timer_callback(self):
        #my_param = self.get_parameter('my_parameter').get_parameter_value().integer_value  # Retrieve the current value of 'my_parameter'
        if not self.config_valid:
            return
        msg = Int16()
        sensor_state = GPIO.input(self.sensor_pin)                                          # Create a new String message
        msg.data = int(not sensor_state)                                     # set msg.data to have the value of my_param
        self.publisher_.publish(msg)                            # Publish the message to the topic
        #self.get_logger().info('Publishing: "%s"' % msg.data)   # Log the published message for debugging


def main(args=None):
    print ("Beginning to talk...")          # Print a starting message
    rclpy.init(args=args)                   # Initialize the ROS 2 Python client library

    sensor_node = SensorNode()  # Create an instance of the SensorNode class

    try:
        rclpy.spin(sensor_node)       # Keep the node active and processing callbacks until interrupted

    except KeyboardInterrupt:   # Handle a keyboard interrupt (Ctrl+C)
        print("\n")             # Print a newline for better format
        print("Stopping...")    # Print a stopping message
 
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        sensor_node.destroy_node()
        if rclpy.ok():                      # Check if the rclpy library is still running
            rclpy.shutdown()                # Shut down the ROS 2 client library, cleanly terminating the node



if __name__ == '__main__':
    main()                  # Call the main function to execute the code when the script is run
    