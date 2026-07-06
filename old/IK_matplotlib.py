import matplotlib.pyplot as plt
import math
from IK_DEMO import plot_robotic_arm

def fk_points_planar(L1, L2, L3, theta_shoulder, theta_elbow, theta_wrist):
    # Takes the angles from the inverse kinematics and returns the final coordinates of each joint.
    # (Forward kinematics for each joint)
    Sx, Sz = 0.0, 0.0

    Ex = L1 * math.cos(theta_shoulder)
    Ez = L1 * math.sin(theta_shoulder)

    Wx = Ex + L2 * math.cos(theta_shoulder + theta_elbow)
    Wz = Ez + L2 * math.sin(theta_shoulder + theta_elbow)

    gamma = theta_shoulder + theta_elbow + theta_wrist
    Tx = Wx + L3 * math.cos(gamma)
    Tz = Wz + L3 * math.sin(gamma)

    return (Sx, Sz), (Ex, Ez), (Wx, Wz), (Tx, Tz)

def draw_arm(ax, L1, L2, L3, target_x, target_z, raw_angles):
    ax.clear()
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("z")
    ax.axhline(0, linewidth=1)
    ax.axvline(0, linewidth=1)

    # View box
    R = L1 + L2 + L3
    ax.set_xlim(-R + 5, R - 5)
    ax.set_ylim(-R + 5, R - 5)

    # Target point
    ax.plot([target_x], [target_z], marker="o", markersize=10,
            markerfacecolor="none", markeredgewidth=2)

    if raw_angles is None:
        ax.set_title("Unreachable target")
        plt.draw()
        return

    tb, ts, te, tw = raw_angles

    # Planar joint points in the arm's reach plane (always +planar_x in IK)
    S, E, W, T = fk_points_planar(L1, L2, L3, ts, te, tw)

    # APPLY base rotation to show left/right in our 2D plot:
    # With y_fixed=0, this mirrors x when tb = pi.
    c = math.cos(tb)

    def apply_base(p):
        return (p[0] * c, p[1])   # x rotated by base, z unchanged

    S = apply_base(S)
    E = apply_base(E)
    W = apply_base(W)
    T = apply_base(T)

    # Links
    ax.plot([S[0], E[0]], [S[1], E[1]], linewidth=6)
    ax.plot([E[0], W[0]], [E[1], W[1]], linewidth=6)
    ax.plot([W[0], T[0]], [W[1], T[1]], linewidth=6)

    # Joints
    ax.plot([S[0]], [S[1]], marker="o", markersize=10, color="blue")
    ax.plot([E[0]], [E[1]], marker="o", markersize=10, color="red")
    ax.plot([W[0]], [W[1]], marker="o", markersize=10, color="green")
    ax.plot([T[0]], [T[1]], marker="o", markersize=8, color="black")

    ax.set_title(
        f"Target (x,z)=({target_x:.2f},{target_z:.2f}) | "
        f"base={math.degrees(tb):.1f}°, shoulder={math.degrees(ts):.1f}°, "
        f"elbow={math.degrees(te):.1f}°, wrist={math.degrees(tw):.1f}°"
    )

    plt.draw()

def main():
    L1, L2, L3 = 10, 7.5, 2.5
    
    z_offset = 0  # keep 0 for this planar demo
    y_fixed = 0   # ignore base, keep y=0

    fig, ax = plt.subplots()

    # Start straight up:
    # Pick a target straight up (x small, z near reach) to force an "up-ish" posture.
    target_x, target_z = 0.0, L1 + L2 - 2.0

    sol = plot_robotic_arm(L1, L2, L3, target_x, y_fixed, target_z, z_offset=z_offset)
    raw_angles = None if sol is None else sol[0]
    draw_arm(ax, L1, L2, L3, target_x, target_z, raw_angles)

    def on_click(event):
        # Ignore clicks outside axes
        if event.inaxes != ax:
            return
        tx, tz = event.xdata, event.ydata

        sol = plot_robotic_arm(L1, L2, L3, tx, y_fixed, tz, z_offset=z_offset)
        raw_angles = None if sol is None else sol[0]
        draw_arm(ax, L1, L2, L3, tx, tz, raw_angles)

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.show()


if __name__ == "__main__":
    main()