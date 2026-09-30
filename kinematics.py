import numpy as np


def forward_kinematics(angles, lengths):
    x, y = 0.0, 0.0          # the base of the arm sits at the origin
    total_angle = 0.0        # running sum of joint angles
    points = [(x, y)]        # we store every joint position, starting with the base

    for theta, L in zip(angles, lengths):
        total_angle += theta                  # this link's absolute direction
        x += L * np.cos(total_angle)          # step along that direction
        y += L * np.sin(total_angle)
        points.append((x, y))                 # save where this link ends

    return np.array(points)


def inverse_kinematics(x, y, lengths, elbow_up=True):
    L1, L2 = lengths
    dist_sq = x**2 + y**2                         # squared distance from base to target

    # law of cosines gives the elbow angle
    cos_elbow = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_elbow = np.clip(cos_elbow, -1.0, 1.0)     # guards against tiny rounding errors
    elbow = np.arccos(cos_elbow)
    if elbow_up:
        elbow = -elbow                            # the mirror-image solution

    # shoulder angle = direction to the target, corrected for the elbow bend
    shoulder = np.arctan2(y, x) - np.arctan2(L2 * np.sin(elbow), L1 + L2 * np.cos(elbow))

    return [shoulder, elbow]


def jacobian(angles, lengths):
    n = len(angles)
    points = forward_kinematics(angles, lengths)
    hand = points[-1]
    J = np.zeros((2, n))
    for i in range(n):
        r = hand - points[i]                  # vector from joint i to the hand
        J[0, i] = -r[1]                       # how hand x changes if joint i rotates
        J[1, i] = r[0]                        # how hand y changes if joint i rotates
    return J


def solve_ik_numeric(target, angles, lengths, iterations=200):
    angles = np.array(angles, dtype=float)
    for _ in range(iterations):
        hand = forward_kinematics(angles, lengths)[-1]
        error = np.array(target) - hand       # how far the hand is from the target
        if np.linalg.norm(error) < 1e-4:
            break
        J = jacobian(angles, lengths)
        angles += np.linalg.pinv(J) @ error * 0.5   # small step toward the target
    return angles
