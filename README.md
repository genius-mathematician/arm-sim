# arm-sim

A simple 2-link and 3-link robotic arm simulation in Python, using NumPy and Matplotlib.

## Run it

    uv sync
    uv run main.py

Click anywhere on the plot and the arm moves to that point.

## Files

- `kinematics.py`: forward kinematics, 2-link inverse kinematics, and a Jacobian-based solver for any number of links
- `draw.py`: draws a static arm
- `animate.py` / `animate3.py`: animate the 2-link and 3-link arms to a target
- `interactive.py`: click-to-move version (this is what `main.py` runs)
