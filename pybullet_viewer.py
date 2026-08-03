import math
import time

import pybullet as p

from config import *
from IK import (
    solve_ik,
    angles_by_name,
    zero_solution,
)


# Find the PyBullet index number for each movable joint.
def find_joint_indices(robot_id):
    joint_indices = {}

    for index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, index)

        joint_name = joint_info[1].decode("utf-8")

        joint_indices[joint_name] = index

    # Make sure all four joints were found.
    for joint_name in JOINT_NAMES:
        if joint_name not in joint_indices:
            raise RuntimeError(
                f"Could not find {joint_name} in the URDF"
            )

    return joint_indices


# Display an IKPy solution on the PyBullet robot.
def apply_solution(robot_id, joint_indices, solution):
    angles = angles_by_name(solution)

    for joint_name in JOINT_NAMES:
        p.resetJointState(
            robot_id,
            joint_indices[joint_name],
            angles[joint_name],
        )


# Print the four joint angles.
def print_solution(target, solution, error):
    angles = angles_by_name(solution)

    print(
        f"\nTarget: "
        f"X={target[0]:.2f}, "
        f"Y={target[1]:.2f}, "
        f"Z={target[2]:.2f} inches"
    )

    for joint_name in JOINT_NAMES:
        angle_degrees = math.degrees(
            angles[joint_name]
        )

        print(
            f"{joint_name:16s}: "
            f"{angle_degrees:9.3f} degrees"
        )

    print(f"Position error: {error:.6f} inches")


# Open and run the PyBullet viewer.
def run_viewer():

    # Open the PyBullet window.
    connection = p.connect(p.GUI)

    if connection < 0:
        raise RuntimeError(
            "Could not open the PyBullet window"
        )

    try:
        p.setGravity(0, 0, 0)

        # Position the camera.
        p.resetDebugVisualizerCamera(
            cameraDistance=1.6,
            cameraYaw=40,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0.2, 0.4],
        )

        # Load the robot.
        robot_id = p.loadURDF(
            str(URDF_PATH),
            useFixedBase=True,
        )

        joint_indices = find_joint_indices(robot_id)

        # Create the red target sphere.
        sphere_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=0.018,
            rgbaColor=[1, 0, 0, 1],
        )

        target_sphere = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=sphere_shape,
        )

        home_x = HOME_TARGET_IN[0]
        home_y = HOME_TARGET_IN[1]
        home_z = HOME_TARGET_IN[2]

        # Create the XYZ sliders.
        x_slider = p.addUserDebugParameter(
            "Target X (inches)",
            -35,
            35,
            home_x,
        )

        y_slider = p.addUserDebugParameter(
            "Target Y (inches)",
            -35,
            45,
            home_y,
        )

        z_slider = p.addUserDebugParameter(
            "Target Z (inches)",
            -20,
            60,
            home_z,
        )

        current_solution = zero_solution()
        desired_solution = zero_solution()

        previous_target = None

        print("PyBullet viewer is running.")

        while p.isConnected():

            # Read the XYZ sliders.
            target = [
                p.readUserDebugParameter(x_slider),
                p.readUserDebugParameter(y_slider),
                p.readUserDebugParameter(z_slider),
            ]

            # Only recalculate IK when a slider changes.
            if target != previous_target:

                solution, error = solve_ik(
                    target,
                    desired_solution,
                )

                # Move the red sphere to the requested target.
                target_meters = [
                    target[0] * INCH_TO_METER,
                    target[1] * INCH_TO_METER,
                    target[2] * INCH_TO_METER,
                ]

                p.resetBasePositionAndOrientation(
                    target_sphere,
                    target_meters,
                    [0, 0, 0, 1],
                )

                # Accept only accurate IK solutions.
                if error <= MAX_IK_ERROR_IN:
                    desired_solution = solution

                    print_solution(
                        target,
                        solution,
                        error,
                    )

                else:
                    print(
                        f"\nTarget rejected. "
                        f"Position error: {error:.3f} inches"
                    )

                previous_target = target.copy()

            # Smoothly move from the current pose to the new pose.
            for index in range(len(current_solution)):
                difference = (
                    desired_solution[index]
                    - current_solution[index]
                )

                current_solution[index] += (
                    difference
                    * ANIMATION_SMOOTHING
                )

            apply_solution(
                robot_id,
                joint_indices,
                current_solution,
            )

            p.stepSimulation()
            time.sleep(1 / 60)

    except KeyboardInterrupt:
        print("\nViewer stopped.")

    finally:
        if p.isConnected():
            p.disconnect()


# Allow this file to be run directly for testing.
if __name__ == "__main__":
    run_viewer()