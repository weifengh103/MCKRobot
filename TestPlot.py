import matplotlib.pyplot as plt
import numpy as np
import time

# Initialize the plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], [])
ax.set_xlim(0, 10)
ax.set_ylim(-1, 1)

# Initial data setup
xdata = np.linspace(0, 10, 100)
ydata = np.sin(xdata)

# Update the plot function
def update_plot(x, y):
    line.set_xdata(x)
    line.set_ydata(y)
    ax.draw_artist(ax.patch)
    ax.draw_artist(line)
    # fig.canvas.update()
    fig.canvas.flush_events()

# Main loop for live update
for i in range(1000):
    update_plot(xdata, np.sin(xdata + i * 0.1))
    time.sleep(0.01)  # Delay to mimic data acquisition

plt.ioff()  # Turn off interactive mode
plt.show()
