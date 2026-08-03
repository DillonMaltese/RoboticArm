import math
from ikpy.chain import Chain

from config import *


# Make sure the URDF exists.
if not URDF_PATH.exists():
    raise FileNotFoundError(
        f"Could not find the URDF file:\n{URDF_PATH}"
    )


# Load the robot model from the URDF.
robot_chain = Chain.from_urdf_file(
    str(URDF_PATH),
    base_elements=["base_link"],
    active_links_mask=ACTIVE_LINKS_MASK,
)


# Create IKPy's complete all-zero starting pose.
def zero_solution():
    return [0.0] * len(robot_chain.links)


# Extract the four movable joint angles from IKPy's solution.
def angles_by_name(solution):
    return {
        "base_joint": float(solution[1]),
        "shoulder_joint": float(solution[2]),
        "elbow_joint": float(solution[3]),
        "wrist_joint": float(solution[4]),
    }


# Calculate the tool-tip XYZ position from the joint angles.
def forward_position_inches(solution):
    result = robot_chain.forward_kinematics(solution)

    return [
        result[0, 3] / INCH_TO_METER,
        result[1, 3] / INCH_TO_METER,
        result[2, 3] / INCH_TO_METER,
    ]


# Calculate joint angles for a requested XYZ position.
def solve_ik(target_inches, starting_angles=None):

    # Make sure the target contains X, Y, and Z.
    if len(target_inches) != 3:
        raise ValueError(
            "target_inches must contain [x, y, z]"
        )

    x = target_inches[0]
    y = target_inches[1]
    z = target_inches[2]

    # IKPy and the URDF use meters.
    target_meters = [
        x * INCH_TO_METER,
        y * INCH_TO_METER,
        z * INCH_TO_METER,
    ]

    # For the first calculation, begin searching from the zero pose.
    if starting_angles is None:
        starting_angles = zero_solution()

    # Calculate the joint angles.
    solution = robot_chain.inverse_kinematics(
        target_position=target_meters,

        # Keep the final tool section vertical and pointing down.
        target_orientation=[0.0, 0.0, 1.0],
        orientation_mode="Z",

        # Search near the robot's previous pose.
        initial_position=starting_angles,

        max_iter=500,
    )

    # Check the answer using forward kinematics.
    calculated_position = forward_position_inches(solution)

    # Calculate the distance between the target and actual result.
    error_inches = math.dist(
        calculated_position,
        target_inches
    )

    return solution, error_inches

# Move straight outward in the direction the base is currently facing.
# Use a negative distance to move backward.
def move_forward(distance_inches, current_solution):

    # Find the current tool-tip position.
    current_position = forward_position_inches(current_solution)

    x = current_position[0]
    y = current_position[1]
    z = current_position[2]

    # Current horizontal distance from the base rotation axis.
    radius = math.hypot(x, y)

    if radius < 0.000001:
        raise ValueError(
            "Cannot determine forward direction while the "
            "tool is directly above the base rotation axis."
        )

    # Direction straight outward from the base.
    forward_x = x / radius
    forward_y = y / radius

    # Move along that direction without changing height.
    new_target = [
        x + distance_inches * forward_x,
        y + distance_inches * forward_y,
        z
    ]

    # Save the normal joint mask.
    original_mask = robot_chain.active_links_mask.copy()

    # Lock the base joint during this calculation.
    robot_chain.active_links_mask[1] = False

    try:
        new_solution, error = solve_ik(
            new_target,
            current_solution
        )
    finally:
        # Always unlock the base again afterward.
        robot_chain.active_links_mask = original_mask

    return new_target, new_solution, error