import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import forward_kinematics, inverse_kinematics


def animate_to(start_angles, target, lengths, frames=60):
    start = np.array(start_angles)
    goal = np.array(inverse_kinematics(target[0], target[1], lengths))

    fig, ax = plt.subplots()
    line, = ax.plot([], [], "-o", linewidth=4, markersize=10)   # the arm, empty for now
    ax.plot(target[0], target[1], "rx", markersize=12)          # red X marks the target

    reach = sum(lengths) + 0.5
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.grid(True)

    def update(i):
        t = i / (frames - 1)                     # goes from 0 to 1 over the animation
        angles = start + t * (goal - start)      # blend from start angles to goal angles
        pts = forward_kinematics(angles, lengths)
        line.set_data(pts[:, 0], pts[:, 1])
        return line,

    anim = FuncAnimation(fig, update, frames=frames, interval=30, repeat=False)
    plt.show()
