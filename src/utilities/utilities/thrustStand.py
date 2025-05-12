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
        self.timer = self.create_timer(0.01, self.read_serial_data)
        self.sequence_timer = self.create_timer(1.0, self.run_float_sequence)
        self.ramp_timer = self.create_timer(1/30, self.run_ramp_sequence)
        self.throttle_value = 0.0

        # Subscriber for activating float sequence
        self.data_collection_activated = False
        self.create_subscription(String, 'activate_float_sequence', self.activate_float_sequence_callback, QoSProfile(depth=3))

        # CSV file setup
        self.csv_file = open('thrust_stand_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Throttle', 'Thrust'])

        self.step_input_values = [0.1, 0.2, 0.3, 0.4, 0.2, 0.4, 0.2, 0.3, 0.2, 0.1]
        #self.step_input_values = [0.1, 0.2, 0.3, 0.1, 0.3, 0.2, 0.1]
        
        self.ramp_mode = False  # New flag to toggle ramp mode
        self.step_iteration = 0
        self.ramp_sample_count = 0

    def read_serial_data(self):
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0



        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
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


        if self.data_collection_activated:
            msg.armed = True
            msg.channel_1 = self.throttle_value

        self.cmd_publisher_.publish(msg)

    def activate_float_sequence_callback(self, msg):
        if msg.data.lower() == 'start' and not self.data_collection_activated:
            self.data_collection_activated = True
            self.sequence_timer.cancel()
            self.sequence_timer.reset()
            self.ramp_timer.cancel()
            self.step_iteration = 0
            self.get_logger().info('Float sequence activated.')
            self.throttle_value = 0.0
        elif msg.data.lower() == 'ramp':

            self.ramp_mode = True
            self.get_logger().info('Ramp mode activated.')
            self.data_collection_activated = True
            self.ramp_timer.cancel()
            self.ramp_timer.reset()
            self.sequence_timer.cancel()
            self.step_iteration = 0
            self.get_logger().info('Float sequence activated.')
            self.throttle_value = 0.0

        elif msg.data.lower() == 'stop':
            self.data_collection_activated = False
            if self.sequence_timer:
                self.sequence_timer.cancel()
            
            if self.ramp_timer:
                self.ramp_timer.cancel()
            # Close the CSV file when data collection ends
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')

    def run_ramp_sequence(self):
        if not self.data_collection_activated:
            return

        if self.step_iteration >= len(self.step_input_values) - 1:
            self.step_iteration = 0
            self.get_logger().info('Ramp sequence completed. Reset to 0.')
            self.ramp_timer.cancel()
            self.data_collection_activated = False
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')
            self.throttle_value = 0.0
            return

        start_value = self.step_input_values[self.step_iteration]
        end_value = self.step_input_values[self.step_iteration + 1]
        step_size = (end_value - start_value) / 30  # 30 samples for 1 second ramp


        self.throttle_value = start_value + step_size * self.ramp_sample_count
        self.ramp_sample_count += 1

        if self.ramp_sample_count >= 30:
            self.step_iteration += 1
            self.ramp_sample_count = 0

    def run_float_sequence(self):
        if not self.data_collection_activated:
            return

        if self.step_iteration >= len(self.step_input_values):
            self.step_iteration = 0
            self.get_logger().info('Step sequence completed. Reset to 0.')
            self.sequence_timer.cancel()
            self.data_collection_activated = False
            self.csv_file.close()
            self.get_logger().info('CSV file saved.')
            self.throttle_value = 0.0
            return

        self.throttle_value = self.step_input_values[self.step_iteration]
        self.step_iteration += 1

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
