#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ToFSerialBridge(Node):
    def __init__(self):
        super().__init__("tof_serial_bridge")

        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baud", 115200)

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)

        self.left_pub = self.create_publisher(Float32, "/tof_left_mm", 10)
        self.right_pub = self.create_publisher(Float32, "/tof_right_mm", 10)

        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.get_logger().info(f"Reading serial from {port} at {baud}")

        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return

        try:
            l_str, r_str = line.split(",")
            l_val = float(l_str)
            r_val = float(r_str)
        except Exception:
            return

        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = l_val
        right_msg.data = r_val

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

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
