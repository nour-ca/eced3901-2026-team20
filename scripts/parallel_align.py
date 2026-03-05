#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


SERIAL_PORT = "/dev/ttyUSB4"   # change if needed
BAUD = 115200

TOLERANCE = 15      # mm difference allowed
MAX_DISTANCE = 200  # ignore readings larger than this


class ParallelAlign(Node):

    def __init__(self):
        super().__init__("parallel_align")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

        self.get_logger().info("Serial connected")

        time.sleep(2)

        self.align_loop()

    def read_sensors(self):

        line = self.ser.readline().decode().strip()

        if "," not in line:
            return None, None

        try:
            left, right = map(int, line.split(","))
        except:
            return None, None

        if left > MAX_DISTANCE or right > MAX_DISTANCE:
            return None, None

        return left, right


    def move(self, linear=0.0, angular=0.0, duration=1.0):

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        start = time.time()

        while time.time() - start < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

        stop = Twist()
        self.cmd_pub.publish(stop)


    def align_loop(self):

        while True:

            left, right = self.read_sensors()

            if left is None:
                continue

            print("LEFT:", left, "RIGHT:", right)

            diff = abs(left - right)

            if diff < TOLERANCE:
                print("Aligned!")
                break

            if left > right:
                print("Adjust left side")

                self.move(-0.1, 0, 1.0)
                self.move(0, 0.3, 1.0)
                self.move(0.1, 0, 1.0)
                self.move(0, -0.3, 1.0)

            else:
                print("Adjust right side")

                self.move(-0.1, 0, 1.0)
                self.move(0, -0.3, 1.0)
                self.move(0.1, 0, 1.0)
                self.move(0, 0.3, 1.0)

            time.sleep(1)

        self.get_logger().info("Alignment complete")


def main():

    rclpy.init()

    node = ParallelAlign()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
