import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import forward_kinematics, solve_ik_numeric


def run_interactive(lengths, start_angles, frames=40):
    state = {
        "current": np.array(start_angles, dtype=float),   # where the arm is right now
        "start": np.array(start_angles, dtype=float),     # pose at the start of a move
        "goal": np.array(start_angles, dtype=float),      # pose it is heading toward
        "frame": frames,                                  # frames == "not moving"
    }

    fig, ax = plt.subplots()
    line, = ax.plot([], [], "-o", linewidth=4, markersize=10)
    marker, = ax.plot([], [], "rx", markersize=12)        # red X follows your click

    reach = sum(lengths) + 0.5
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_title("Click anywhere to move the arm")

    def on_click(event):
        if event.inaxes != ax:                            # ignore clicks outside the plot
            return
        target = (event.xdata, event.ydata)
        marker.set_data([target[0]], [target[1]])
        state["start"] = state["current"].copy()
        state["goal"] = solve_ik_numeric(target, state["current"], lengths)
        state["frame"] = 0                                # restart the glide

    def update(_):
        if state["frame"] < frames:
            t = state["frame"] / (frames - 1)
            state["current"] = state["start"] + t * (state["goal"] - state["start"])
            state["frame"] += 1
        pts = forward_kinematics(state["current"], lengths)
        line.set_data(pts[:, 0], pts[:, 1])
        return line, marker

    fig.canvas.mpl_connect("button_press_event", on_click)
    anim = FuncAnimation(fig, update, interval=30, cache_frame_data=False)
    plt.show()
