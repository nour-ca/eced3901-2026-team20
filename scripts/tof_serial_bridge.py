#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class ToFSerialBridge(Node):
    def __init__(self):
        super().__init__("tof_serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_left", "tof_left")
        self.declare_parameter("frame_right", "tof_right")
        self.declare_parameter("min_m", 0.02)
        self.declare_parameter("max_m", 4.0)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)

        self.pub_l = self.create_publisher(Range, "/tof_left", 10)
        self.pub_r = self.create_publisher(Range, "/tof_right", 10)

        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.get_logger().info(f"Reading ToF CSV from {port} @ {baud}")

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def _msg(self, frame_id: str, meters: float) -> Range:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.5
        msg.min_range = float(self.get_parameter("min_m").value)
        msg.max_range = float(self.get_parameter("max_m").value)
        msg.range = meters
        return msg

    def tick(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return
        try:
            l_s, r_s = line.split(",")
            l_mm = int(l_s)
            r_mm = int(r_s)
        except Exception:
            return

        l_m = max(0.02, min(4.0, l_mm / 1000.0))
        r_m = max(0.02, min(4.0, r_mm / 1000.0))

        self.pub_l.publish(self._msg(self.get_parameter("frame_left").value, l_m))
        self.pub_r.publish(self._msg(self.get_parameter("frame_right").value, r_m))

def main():
    rclpy.init()
    node = ToFSerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
