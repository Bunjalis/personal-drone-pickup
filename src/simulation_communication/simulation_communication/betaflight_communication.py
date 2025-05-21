import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from interfaces.msg import MotionCaptureState, ELRSCommand, Telemetry  # Import the Telemetry message


class BetaflightInterfaceNode(Node):
    def __init__(self):
        super().__init__('betaflight_interface')
        self.declare_parameter('udp_ip', '127.0.0.1')
        self.declare_parameter('udp_port', 9002)
        self.declare_parameter('num_motors', 4)
        self.declare_parameter('imu_udp_port', 9003)  # Port for IMU data to Betaflight
        self.declare_parameter('cmd_udp_port', 9004)
        self.udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.num_motors = self.get_parameter('num_motors').get_parameter_value().integer_value
        self.imu_udp_port = self.get_parameter('imu_udp_port').get_parameter_value().integer_value
        self.cmd_udp_port = self.get_parameter('cmd_udp_port').get_parameter_value().integer_value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.publisher = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)
        self.get_logger().info(f"Listening for Betaflight SITL on {self.udp_ip}:{self.udp_port}")

        self.subscription_control = self.create_subscription(ELRSCommand, 'ELRSCommand', self.controller_commands_callback, 10)

        self.last_timestep = 0.0


        self.timer = self.create_timer(0.001, self.listen_udp)
        # Subscribe to IMU topic (bridged to ROS2)
        self.create_subscription(
            Imu,
            '/world/quadcopter/model/x3/link/X3/base_link/sensor/imu_sensor/imu',
            self.imu_callback,
            10
        )

    def listen_udp(self):
        self.sock.setblocking(False)

        try:
            data, addr = self.sock.recvfrom(1024)
            if len(data) >= self.num_motors * 4:
                motors = struct.unpack('<' + 'f'*self.num_motors, data[:self.num_motors*4])
                motor_speeds = [float(m) for m in motors]
                max_rot_val = 4631.0
                self.speed = [motor_speeds[3]*max_rot_val,motor_speeds[0]*max_rot_val,motor_speeds[1]*max_rot_val,motor_speeds[2]*max_rot_val]
                actuator_msg = Actuators()
                actuator_msg.header.stamp = self.get_clock().now().to_msg()
                actuator_msg.velocity = self.speed

                #print("Actuator command: ", actuator_msg.velocity)
                self.publisher.publish(actuator_msg)


            else:
                self.get_logger().warn(f"Received packet of unexpected size: {len(data)} bytes")
        except BlockingIOError:
            pass

    def imu_callback(self, msg: Imu):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Print IMU callrate estimation
        if hasattr(self, 'last_timestep') and self.last_timestep != 0.0:
            dt = timestamp - self.last_timestep
            if dt > 0:
                callrate = 1.0 / dt
                self.get_logger().info(f"Estimated IMU callrate: {callrate:.2f} Hz")
        # Convert gyro and accel from ENU (Gazebo) to NED (Betaflight): invert Z
        gyro_x = float(msg.angular_velocity.x)
        gyro_y = -float(msg.angular_velocity.y)
        gyro_z = -float(msg.angular_velocity.z)
        accel_x = float(msg.linear_acceleration.z)  # Gazebo is already in NED
        accel_y = -float(msg.linear_acceleration.z)  # Gazebo is already in NED
        accel_z = -float(msg.linear_acceleration.z)  # Gazebo is already in NED
        # Correct quaternion mapping: [qy, qx, -qz, qw] (pitch, roll, -yaw, w)
        qx_enu = float(msg.orientation.x)
        qy_enu = float(msg.orientation.y)
        qz_enu = float(msg.orientation.z)
        qw_enu = float(msg.orientation.w)
        qw = qw_enu
        qx = qx_enu  # pitch (inverted to fix pitch direction)
        qy = -qy_enu   # roll
        qz = -qz_enu  # yaw
        vel_x = 0.0
        vel_y = 0.0
        vel_z = 0.0
        pos_x = 0.0
        pos_y = 0.0
        pos_z = 0.0
        pressure = 0.0
        imu_packet = struct.pack(
            '<18d',
            timestamp,
            gyro_x, gyro_y, gyro_z,
            accel_x, accel_y, accel_z,
            qw, qx, qy, qz,  # C++ order: w, x, y, z
            vel_x, vel_y, vel_z,
            pos_x, pos_y, pos_z,
            pressure
        )
        self.imu_sock.sendto(imu_packet, (self.udp_ip, self.imu_udp_port))

        self.last_timestep = timestamp



    def controller_commands_callback(self, msg):
        channels = [0] * 16

        for i in range(0, 16):
            channels[i] = 1500


        channels[0] = int( msg.channel_0 * 500 )  + 1500  # roll
        channels[1] = int( msg.channel_1 * 500 )  + 1500  # pitch
        channels[2] = int( msg.channel_2 * 1000 ) + 1000  # Throttle
        channels[3] = int( msg.channel_3 * 500 )  + 1500  # yaw

        channels[4] = 2000 if msg.armed else 1000  # aux1 (arming)


        cmd_packet = struct.pack('<d16H', self.last_timestep, *channels)
        self.cmd_sock.sendto(cmd_packet, (self.udp_ip, self.cmd_udp_port))



def main(args=None):
    rclpy.init(args=args)
    node = BetaflightInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
