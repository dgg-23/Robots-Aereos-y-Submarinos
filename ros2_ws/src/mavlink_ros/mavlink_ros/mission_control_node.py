#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from pymavlink import mavutil


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        self.mavlink_url = 'udp:127.0.0.1:14551'  # distinto al battery node para no pisarse
        self.alt_m = 10.0
        self.step_deg = 0.00100   # tamaño del cuadrado (en grados)
        self.dwell_s = 20.0        # segundos en cada vértice

        self.gps_pub = self.create_publisher(NavSatFix, 'target_gps', 10)
        self.batt_sub = self.create_subscription(Float32, 'battery_status', self.on_battery, 10)
        self.landing_sent = False

        # MAVLink
        self.get_logger().info(f"Connecting MAVLink: {self.mavlink_url}")
        self.conn = mavutil.mavlink_connection(self.mavlink_url)
        self.conn.wait_heartbeat()
        self.get_logger().info("MAVLink heartbeat OK")

        # GUIDED + ARM + TAKEOFF
        self.set_mode('GUIDED')
        time.sleep(1)
        self.arm()
        time.sleep(1)
        self.takeoff(self.alt_m)

        # Centro = posición actual tras despegar un poco
        time.sleep(2)
        center_lat, center_lon = self.get_current_latlon()
        self.get_logger().info(f"Center: lat={center_lat:.7f}, lon={center_lon:.7f}")

        # Cuadrado simple en grados (sin conversión a metros)
        # Recorre: esquina inferior-izq -> inferior-der -> superior-der -> superior-izq
        A = (center_lat, center_lon, self.alt_m)
        B = (center_lat + self.step_deg, center_lon, self.alt_m)
        C = (center_lat + self.step_deg, center_lon + self.step_deg, self.alt_m)
        D = (center_lat, center_lon + self.step_deg, self.alt_m)

        self.waypoints = [A, B, C, D]
        self.idx = 0

        # Publica primer vértice y luego rota por los 4
        self.publish_target(self.waypoints[self.idx])
        self.timer = self.create_timer(self.dwell_s, self.step_square)

    def on_battery(self, msg: Float32):
        if self.landing_sent:
            return
        if msg.data <= 20.0:
            self.get_logger().warn("Battery <= 20% -> LAND")
            self.land()
            self.landing_sent = True

    def step_square(self):
        if self.landing_sent:
            return
        self.idx = (self.idx + 1) % len(self.waypoints)
        self.publish_target(self.waypoints[self.idx])

    def publish_target(self, wp):
        lat, lon, alt = wp
        m = NavSatFix()
        m.latitude = float(lat)
        m.longitude = float(lon)
        m.altitude = float(alt)
        self.gps_pub.publish(m)
        self.get_logger().info(f"Published target_gps: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}")


    def set_mode(self, mode):
        self.conn.set_mode(mode)
        self.get_logger().info(f"Mode set to {mode}")

    def arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("ARM command sent")

    def takeoff(self, alt_m):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(alt_m)
        )
        self.get_logger().info(f"TAKEOFF command sent: {alt_m} m")

    def land(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("LAND command sent")

    def get_current_latlon(self):
        # Espera hasta que llegue un GLOBAL_POSITION_INT
        while True:
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg:
                return msg.lat / 1e7, msg.lon / 1e7


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

