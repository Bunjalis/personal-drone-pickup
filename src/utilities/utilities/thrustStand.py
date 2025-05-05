import rclpy
from rclpy.node import Node
import serial
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, String
from rclpy.duration import Duration
from rclpy.timer import Timer
import csv
from interfaces.msg import MotionCaptureState, ELRSCommand
import time

class SerialSubscriberNode(Node):
    def __init__(self):
        super().__init__('serial_subscriber_node')
        self.get_logger().info('Serial Subscriber Node has been started.')

        # Initialize serial port with reduced timeout
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)  # Reduced timeout

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', qos_profile)

        # Timer to periodically read from the serial port
        self.timer = self.create_timer(0.001, self.read_serial_data)
        self.sequence_timer = self.create_timer(1.0, self.run_float_sequence)
        self.throttle_value = 0.0

        # Subscriber for activating float sequence
        self.data_collection_activated = False
        self.create_subscription(String, 'activate_float_sequence', self.activate_float_sequence_callback, QoSProfile(depth=3))

        # CSV file setup
        self.csv_file = open('thrust_stand_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Throttle', 'Thrust'])

        self.step_input_values = [0.1, 0.3, 0.0] 
        self.step_iteration = 0

    def read_serial_data(self):
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

        try:
            if self.serial_port.in_waiting > 0:
                start_time = time.time()  # Start timing the read operation
                line = self.serial_port.readline().decode('utf-8').strip()
                elapsed_time = time.time() - start_time
                self.get_logger().info(f"Serial read took {elapsed_time:.6f} seconds.")

                value = float(line)
                print(f"Throttle: {self.throttle_value}, Thrust: {value}")

                # Save throttle and thrust to CSV if float sequence is active
                if self.data_collection_activated:
                    self.csv_writer.writerow([self.throttle_value, value])

                # Flush/clear everything in the serial input buffer
                self.serial_port.reset_input_buffer()
        except ValueError:
            self.get_logger().warn('Received invalid data that could not be converted to float.')
        except Exception as e:
            self.get_logger().error(f'Error reading from serial port: {e}')

    def activate_float_sequence_callback(self, msg):
        if msg.data.lower() == 'start' and not self.data_collection_activated:
            self.data_collection_activated = True
            self.sequence_timer.cancel()
            self.sequence_timer.reset()
            self.step_iteration = 0
            self.get_logger().info('Float sequence activated.')
            self.throttle_value = 0.0

        elif msg.data.lower() == 'stop':
            self.data_collection_activated = False
            if self.sequence_timer:
                self.sequence_timer.cancel()
            # Close the CSV file when data collection ends
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')

    def run_float_sequence(self):
        if not self.data_collection_activated:
            return

        mode = "Step input"
        if mode == "Step input":

            if self.step_iteration >= len(self.step_input_values):
                self.step_iteration = 0
                self.get_logger().info('Sequence completed. Reset to 0.')
                self.sequence_timer.cancel()
                self.data_collection_activated = False
                # Close the CSV file when data collection ends
                self.csv_file.close()
                self.get_logger().info('CSV file saved.')
                self.throttle_value = 0.0
                return

            self.throttle_value = self.step_input_values[self.step_iteration]
            self.step_iteration += 1

        else:
            self.throttle_value += 0.05
            if self.throttle_value > 1.01:
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
