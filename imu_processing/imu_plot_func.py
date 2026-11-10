import signal
import threading
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import SingleThreadedExecutor


class ImuPlotFunc:
    def __init__(self, window=200, title="IMU *variable* Data Plot", ylabel="variable"):
        self.t = deque(maxlen=window)
        self.x_d = deque(maxlen=window)
        self.y_d = deque(maxlen=window)
        self.z_d = deque(maxlen=window)

        plt.ion()
        self.fig = plt.figure()    # Create a new window for this plot

        # Create the three lines, label the axes, and add a title
        self.line_x, = plt.plot([], [], label="X")
        self.line_y, = plt.plot([], [], label="Y")
        self.line_z, = plt.plot([], [], label="Z")
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)

        plt.title(title)
        plt.legend()
        plt.grid(True)

    def add_sample(self, t, x, y, z):
        """Store one reading. Call this from your IMU callback."""
        self.t.append(t)
        self.x_d.append(x)
        self.y_d.append(y)
        self.z_d.append(z)

    def update(self):
        """Redraw the plot. Call this from a timer, not every callback."""
        if not self.t:
            return

        plt.figure(self.fig.number)    # Switch to this plot's window

        self.line_x.set_data(self.t, self.x_d)
        self.line_y.set_data(self.t, self.y_d)
        self.line_z.set_data(self.t, self.z_d)

        # Rescale to fit the new data
        plt.gca().relim()
        plt.gca().autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self):
        plt.close(self.fig)


def spin_with_plots(node, redraw, rate_hz=10.0):
    """Spin a node in the background and redraw plots in the main thread.

    matplotlib must own the main thread to stay responsive. Call
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO) first: rclpy's
    own Ctrl+C handler raises inside Tk callbacks and leaves plt.pause() stuck.
    Returns when Ctrl+C is pressed or every plot window is closed.
    """
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while not stop.is_set() and rclpy.ok() and plt.get_fignums():
            redraw()
            # Process window events until the next frame. Not plt.pause(): it
            # forces a full redraw of the figure every call, which is slow
            plt.figure(plt.get_fignums()[0]).canvas.start_event_loop(1.0 / rate_hz)
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)

