#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Use the stable by-id path you saw earlier (CH340 / 1a86)
PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD = 115200

class ToFBridge(Node):
    def __init__(self):
        super().__init__("tof_bridge")

        self.left_pub = self.create_publisher(Float32, "/tof_left_mm", 10)
        self.right_pub = self.create_publisher(Float32, "/tof_right_mm", 10)

        self.ser = serial.Serial(PORT, BAUD, timeout=0.2)
        self.get_logger().info(f"Reading {PORT} @ {BAUD}")

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return

        # Expected format: left_mm,right_mm
        try:
            l_s, r_s = line.split(",")
            l_mm = float(l_s)
            r_mm = float(r_s)
        except Exception:
            return

        l = Float32(); l.data = l_mm
        r = Float32(); r.data = r_mm
        self.left_pub.publish(l)
        self.right_pub.publish(r)

def main():
    rclpy.init()
    node = ToFBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
