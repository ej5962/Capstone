import matplotlib.pyplot as plt
from collections import deque


class ImuPlot:
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