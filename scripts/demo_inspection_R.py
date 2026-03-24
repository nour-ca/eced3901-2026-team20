#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy
import time
import serial
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


def send_servo_command(port='/dev/ttyACM0', baud=115200, command='MOVE_BOTH\n'):
    """
    Open serial connection to Arduino and send a command.
    Change the port if your Arduino shows up somewhere else.
    """
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)  # give Arduino time to reset after serial opens
        ser.write(command.encode('utf-8'))
        ser.flush()
        print(f'Sent servo command to Arduino: {command.strip()}')
    except serial.SerialException as e:
        print(f'Failed to communicate with Arduino on {port}: {e}')
    finally:
        if ser is not None and ser.is_open:
            ser.close()


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Waypoints: [x, y, yaw]
    inspection_route = [
        [3.31, 0.0, 0.0]
    ]

    # Wait for Nav2 to fully activate
    navigator.waitUntilNav2Active()

    # Build waypoint list
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'

    for pt in inspection_route:
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_pose.pose.position.z = 0.0

        q = get_quaternion_from_euler(0.0, 0.0, pt[2])
        inspection_pose.pose.orientation.x = q[0]
        inspection_pose.pose.orientation.y = q[1]
        inspection_pose.pose.orientation.z = q[2]
        inspection_pose.pose.orientation.w = q[3]

        inspection_points.append(deepcopy(inspection_pose))

    # Start waypoint following
    navigator.followWaypoints(inspection_points)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(inspection_points))
            )

    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Inspection complete! Triggering Arduino servos...')
        send_servo_command(
            port='/dev/ttyUSB4',   # change this if needed
            baud=115200,
            command='MOVE_BOTH\n'
        )

    elif result == TaskResult.CANCELED:
        print('Inspection was canceled.')

    elif result == TaskResult.FAILED:
        print('Inspection failed.')

    else:
        print('Inspection ended with unknown result.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
