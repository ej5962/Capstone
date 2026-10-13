#!/usr/bin/env python3

# Keyboard-controlled magnetometer recorder.
#   s : start / stop recording (each start creates a new CSV file)
#   q : quit
#
# Run directly in a terminal (ros2 run), not from a launch file, so the node
# can read key presses.
#
# Parameters:
#   mag_topic  (default /imu/mag)   sensor_msgs/msg/MagneticField
#   output_dir (default ~/mag_recordings)

# Python imports
import os
import sys
import csv
import select
import termios
import threading
import tty
from datetime import datetime

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagRecorder(Node):

    def __init__(self):
        super().__init__('mag_recorder')

        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/mag_recordings'))

        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # Recording state (shared with the keyboard thread, so use a lock)
        self.lock = threading.Lock()
        self.recording = False
        self.running = True
        self.mag_file = None
        self.mag_writer = None
        self.mag_count = 0

        # Subscription
        self.create_subscription(MagneticField, '/imu/mag' , self.mag_callback, 50)

        # Keyboard thread
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info("Listening on /imu/mag")
        self.get_logger().info("Press 's' to start/stop recording, 'q' to quit.")

    # Keyboard handling
    def keyboard_loop(self):
        if not sys.stdin.isatty():                                                                  # Check if terminal input is available
            self.get_logger().error("No terminal input. Run with 'ros2 run', not a launch file.")
            return

        fd = sys.stdin.fileno()                                # input reader file descriptor
        old_settings = termios.tcgetattr(fd)                   # Save terminal settings
        try:
            tty.setcbreak(fd)                                  # Read single keys, no Enter needed
            while self.running and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    key = sys.stdin.read(1).lower()
                    if key == 's':
                        self.toggle_recording()
                    elif key == 'q':
                        self.running = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)   # Restore terminal

    # Start / stop
    def toggle_recording(self):
        with self.lock:
            if not self.recording:
                self.start_recording()
            else:
                self.stop_recording()

    def start_recording(self):
        # Call with self.lock held
        name = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.output_dir, f'mag_{name}.csv')

        self.mag_file = open(path, 'w', newline='')
        self.mag_writer = csv.writer(self.mag_file)
        self.mag_writer.writerow(['t_stamp', 't_recv', 'mag_x', 'mag_y', 'mag_z'])

        self.mag_count = 0
        self.recording = True
        self.get_logger().info(f"RECORDING STARTED -> {path}")

    def stop_recording(self):
        # Call with self.lock held
        self.recording = False
        if self.mag_file is not None:
            self.mag_file.close()
        self.mag_file = None
        self.mag_writer = None
        self.get_logger().info(f"RECORDING STOPPED: {self.mag_count} rows saved.")

    # Callback
    def mag_callback(self, msg):
        with self.lock:
            if not self.recording:
                return
            stamp = msg.header.stamp
            t_stamp = stamp.sec + stamp.nanosec * 1e-9                 # Time of reading
            t_recv = self.get_clock().now().nanoseconds * 1e-9         # Time received
            m = msg.magnetic_field
            self.mag_writer.writerow([t_stamp, t_recv, m.x, m.y, m.z])
            self.mag_count += 1

    def shutdown(self):
        self.running = False
        with self.lock:
            if self.recording:
                self.stop_recording()
        self.key_thread.join(timeout=1.0)             # Let the terminal be restored


def main(args=None):
    rclpy.init(args=args)
    node = MagRecorder()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()                               # Closes the file if still recording
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()