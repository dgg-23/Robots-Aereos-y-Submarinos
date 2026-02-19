#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from pymavlink import mavutil


class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')

        # Publisher: batería
        self.batt_pub = self.create_publisher(Float32, 'battery_status', 10)

        # Subscriber: objetivo GPS
        self.target_sub = self.create_subscription(
            NavSatFix, 'target_gps', self.target_cb, 10
        )

        # MAVLink a SITL
        self.conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.conn.wait_heartbeat()
        self.get_logger().info("Connected to SITL (14550).")

        # Timer batería (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_battery)

    def publish_battery(self):
        # Opción más estable en SITL: SYS_STATUS (battery_remaining)
        msg = self.conn.recv_match(type='SYS_STATUS', blocking=False)
        if msg and msg.battery_remaining is not None and msg.battery_remaining != -1:
            remaining = float(msg.battery_remaining)
            self.get_logger().info(f"Battery: {remaining:.0f}%")
            self.batt_pub.publish(Float32(data=remaining))

    def target_cb(self, msg: NavSatFix):
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude) if msg.altitude is not None else 10.0

        self.get_logger().info(f"Received target_gps: Lat={lat}, Lon={lon}, Alt={alt}")
        self.send_gps_to_drone(lat, lon, alt)

    def send_gps_to_drone(self, lat, lon, alt):
        self.conn.mav.set_position_target_global_int_send(
            0,
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame of reference
            0b110111111000, # ignora vel/acc/yaw
            int(lat * 1e7), # Latitude as integer
            int(lon * 1e7), # Longitude as integer
            float(alt), # Desired altitude
            0, 0, 0, # Velocities
            0, 0, 0, # Accelerations
            0, 0 # Yaw and rate
        )
        self.get_logger().info("Setpoint sent to drone.")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

