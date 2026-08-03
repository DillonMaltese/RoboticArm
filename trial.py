from pathlib import Path
import math
import time

import numpy as np
import pybullet as p
from ikpy.chain import Chain


INCH_TO_METER = 0.0254
URDF_PATH = Path(__file__).with_name("jarvis.urdf")

JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
]


# ============================================================
# LOAD IK MODEL
# ============================================================

chain = Chain.from_urdf_file(
    str(URDF_PATH),
    base_elements=["base_link"],
    active_links_mask=[
        False,  # Base link
        True,   # Base
        True,   # Shoulder
        True,   # Elbow
        True,   # Wrist
        False,  # Tool-tip fixed joint
    ],
)


def solve_ik(target_inches, starting_angles):
    """Calculate the robot angles for an XYZ target in inches."""

    target_meters = np.asarray(
        target_inches,
        dtype=float,
    ) * INCH_TO_METER

    solution = chain.inverse_kinematics(
        target_position=target_meters,

        # Keep the physical tool vertical downward.
        target_orientation=np.array([0.0, 0.0, 1.0]),
        orientation_mode="Z",

        initial_position=starting_angles,
        max_iter=500,
    )

    # Verify that IK actually reached the requested point.
    calculated_position = (
        chain.forward_kinematics(solution)[:3, 3]
    )

    error_inches = (
        np.linalg.norm(calculated_position - target_meters)
        / INCH_TO_METER
    )

    return solution, target_meters, error_inches


def angles_by_name(solution):
    """Convert IKPy's angle array into a joint-name dictionary."""

    return {
        link.name: float(angle)
        for link, angle in zip(chain.links, solution)
    }


# ============================================================
# OPEN PYBULLET
# ============================================================

p.connect(p.GUI)
p.setGravity(0.0, 0.0, 0.0)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=40,
    cameraPitch=-25,
    cameraTargetPosition=[0.0, 0.2, 0.4],
)

robot_id = p.loadURDF(
    str(URDF_PATH),
    useFixedBase=True,
)


# Match each URDF joint name to its PyBullet joint number.
pybullet_joints = {}

for index in range(p.getNumJoints(robot_id)):
    joint_name = (
        p.getJointInfo(robot_id, index)[1]
        .decode("utf-8")
    )

    pybullet_joints[joint_name] = index


# ============================================================
# TARGET SPHERE
# ============================================================

sphere_visual = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.018,
    rgbaColor=[1.0, 0.0, 0.0, 1.0],
)

target_sphere = p.createMultiBody(
    baseMass=0.0,
    baseVisualShapeIndex=sphere_visual,
)


# ============================================================
# XYZ SLIDERS
# ============================================================

x_slider = p.addUserDebugParameter(
    "Target X (inches)",
    -35.0,
    35.0,
    0.0,
)

y_slider = p.addUserDebugParameter(
    "Target Y (inches)",
    -35.0,
    45.0,
    16.0,
)

z_slider = p.addUserDebugParameter(
    "Target Z (inches)",
    0.0,
    60.0,
    21.5,
)


# ============================================================
# MAIN LOOP
# ============================================================

current_solution = np.zeros(len(chain.links))
desired_solution = current_solution.copy()

previous_target = None

try:
    while p.isConnected():

        target_inches = np.array([
            p.readUserDebugParameter(x_slider),
            p.readUserDebugParameter(y_slider),
            p.readUserDebugParameter(z_slider),
        ])

        target_changed = (
            previous_target is None
            or np.linalg.norm(
                target_inches - previous_target
            ) > 0.001
        )

        if target_changed:
            solution, target_meters, error = solve_ik(
                target_inches,
                desired_solution,
            )

            p.resetBasePositionAndOrientation(
                target_sphere,
                target_meters.tolist(),
                [0.0, 0.0, 0.0, 1.0],
            )

            if error <= 0.05:
                desired_solution = solution
                joint_angles = angles_by_name(solution)

                print(
                    f"\nTarget: "
                    f"X={target_inches[0]:.2f}, "
                    f"Y={target_inches[1]:.2f}, "
                    f"Z={target_inches[2]:.2f} inches"
                )

                for name in JOINT_NAMES:
                    print(
                        f"{name:16s}: "
                        f"{math.degrees(joint_angles[name]):9.3f}°"
                    )

                print(f"Position error: {error:.6f} inches")

            else:
                print(
                    f"\nTarget rejected. "
                    f"IK error was {error:.3f} inches."
                )

            previous_target = target_inches.copy()

        # Smoothly animate toward the new pose.
        current_solution += (
            0.15
            * (desired_solution - current_solution)
        )

        displayed_angles = angles_by_name(current_solution)

        for name in JOINT_NAMES:
            p.resetJointState(
                robot_id,
                pybullet_joints[name],
                displayed_angles[name],
            )

        p.stepSimulation()
        time.sleep(1.0 / 60.0)

except KeyboardInterrupt:
    pass

finally:
    if p.isConnected():
        p.disconnect()