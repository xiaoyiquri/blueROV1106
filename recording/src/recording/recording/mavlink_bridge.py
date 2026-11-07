from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import UInt16MultiArray
import threading


NUM_SERVO_CHANNELS = 8


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        self.imu_pub = self.create_publisher(Imu, 'imu_px4/data', 10)
        self.baro_pub = self.create_publisher(FluidPressure, 'barometer/pressure', 10)
        self.motor_pub = self.create_publisher(UInt16MultiArray, 'rov/motor_pwm', 10)

        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14600')
        self.master.wait_heartbeat()
        self.get_logger().info('MAVLINK connected.')

        self.stop_thread = False
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        while not self.stop_thread:
            try:
                msg = self.master.recv_match(
                    type=['SCALED_IMU2', 'SCALED_PRESSURE', 'SERVO_OUTPUT_RAW'],
                    blocking=True, timeout=1.0)
                if not msg:
                    continue

                msg_type = msg.get_type()
                if msg_type == 'SCALED_IMU2':
                    self._publish_imu(msg)
                elif msg_type == 'SCALED_PRESSURE':
                    self._publish_baro(msg)
                elif msg_type == 'SERVO_OUTPUT_RAW':
                    self._publish_motor(msg)

            except Exception as e:
                self.get_logger().error(f"Error reading MAVLink data: {e}")

    def _publish_imu(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        # MAVLink SCALED_IMU2 uses mG; sensor_msgs/Imu requires m/s^2.
        imu_msg.linear_acceleration.x = msg.xacc * 9.80665 / 1000.0
        imu_msg.linear_acceleration.y = msg.yacc * 9.80665 / 1000.0
        imu_msg.linear_acceleration.z = msg.zacc * 9.80665 / 1000.0
        imu_msg.angular_velocity.x = msg.xgyro / 1000.0
        imu_msg.angular_velocity.y = msg.ygyro / 1000.0
        imu_msg.angular_velocity.z = msg.zgyro / 1000.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        # This message contains no attitude estimate. ROS convention: unavailable.
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

    def _publish_baro(self, msg):
        if msg.press_abs is None:
            return
        fmsg = FluidPressure()
        fmsg.header.stamp = self.get_clock().now().to_msg()
        fmsg.header.frame_id = 'barometer'
        fmsg.fluid_pressure = float(msg.press_abs) * 100.0
        fmsg.variance = 0.0
        self.baro_pub.publish(fmsg)

    def _publish_motor(self, msg):
        if msg.port != 0:
            return
        pwm_msg = UInt16MultiArray()
        pwm_msg.data = [
            getattr(msg, f'servo{i}_raw', 0) for i in range(1, NUM_SERVO_CHANNELS + 1)
        ]
        self.motor_pub.publish(pwm_msg)

    def destroy_node(self):
        self.stop_thread = True
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
