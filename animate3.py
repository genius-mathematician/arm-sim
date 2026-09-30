import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import forward_kinematics, solve_ik_numeric


def animate_to_3link(start_angles, target, lengths, frames=60):
    start = np.array(start_angles, dtype=float)
    goal = solve_ik_numeric(target, start, lengths)     # numeric solver finds the goal pose

    fig, ax = plt.subplots()
    line, = ax.plot([], [], "-o", linewidth=4, markersize=10)
    ax.plot(target[0], target[1], "rx", markersize=12)  # red X marks the target

    reach = sum(lengths) + 0.5
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.grid(True)

    def update(i):
        t = i / (frames - 1)
        angles = start + t * (goal - start)             # blend from start pose to goal pose
        pts = forward_kinematics(angles, lengths)
        line.set_data(pts[:, 0], pts[:, 1])
        return line,

    anim = FuncAnimation(fig, update, frames=frames, interval=30, repeat=False)
    plt.show()
