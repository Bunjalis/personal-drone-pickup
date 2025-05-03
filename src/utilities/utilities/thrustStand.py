import rclpy
from rclpy.node import Node
import serial
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from rclpy.duration import Duration
from rclpy.timer import Timer
import csv
from interfaces.msg import MotionCaptureState, ELRSCommand


class SerialSubscriberNode(Node):
    def __init__(self):
        super().__init__('serial_subscriber_node')
        self.get_logger().info('Serial Subscriber Node has been started.')

        # Initialize serial port
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)

        # Timer to periodically read from the serial port
        self.timer = self.create_timer(0.001, self.read_serial_data)
        self.sequence_timer = self.create_timer(3.0, self.run_float_sequence)
        self.throttle_value = 0.0

        # Publisher for float values
        self.float_publisher = self.create_publisher(Float32, 'float_values', QoSProfile(depth=10))

        # Subscriber for activating float sequence
        self.data_collection_activated = False
        self.create_subscription(String, 'activate_float_sequence', self.activate_float_sequence_callback, QoSProfile(depth=10))


        # CSV file setup
        self.csv_file = open('thrust_stand_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Throttle', 'Thrust'])

    def read_serial_data(self):
        
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                value = float(line)
                
                # Save throttle and thrust to CSV if float sequence is active
                if self.data_collection_activated:
                    self.csv_writer.writerow([self.throttle_value, value])
        except ValueError:
            self.get_logger().warn('Received invalid data that could not be converted to float.')
        except Exception as e:
            self.get_logger().error(f'Error reading from serial port: {e}')



        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0

        if self.data_collection_activated:
            msg.armed = True
            msg.channel_1 = self.throttle_value
        
        self.cmd_publisher_.publish(msg)



    def activate_float_sequence_callback(self, msg):
        if msg.data.lower() == 'start' and not self.data_collection_activated:
            self.data_collection_activated = True
            self.get_logger().info('Float sequence activated.')
        elif msg.data.lower() == 'stop':
            self.data_collection_activated = False
            if self.sequence_timer:
                self.sequence_timer.cancel()
            self.float_publisher.publish(Float32(data=0.0))
            self.get_logger().info('Float sequence stopped and reset to 0.')
            # Close the CSV file when data collection ends
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')

    def run_float_sequence(self):
        if not self.data_collection_activated:
            return

        self.float_publisher.publish(Float32(data=self.throttle_value))
        self.get_logger().info(f'Published value: {self.throttle_value}')

        self.throttle_value += 0.05
        if self.throttle_value > 1.01:
            self.float_publisher.publish(Float32(data=0.0))
            self.get_logger().info('Sequence completed. Reset to 0.')
            self.sequence_timer.cancel()
            self.data_collection_activated = False
            # Close the CSV file when data collection ends
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Serial Subscriber Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
