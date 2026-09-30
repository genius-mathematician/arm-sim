import numpy as np
import matplotlib.pyplot as plt
from kinematics import forward_kinematics


def draw_arm(angles, lengths):
    points = forward_kinematics(angles, lengths)   # base, joint 1, hand

    fig, ax = plt.subplots()
    ax.plot(points[:, 0], points[:, 1], "-o", linewidth=4, markersize=10)

    reach = sum(lengths) + 0.5                     # a little margin around the arm
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")                         # keeps link lengths looking true
    ax.grid(True)
    plt.show()
