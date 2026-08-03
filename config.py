from pathlib import Path

SERIAL_PORT = "COM4"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1

# Folder containing this file
PROJECT_FOLDER = Path(__file__).resolve().parent

# Robot description
URDF_PATH = PROJECT_FOLDER / "jarvis.urdf"

# Unit conversion
INCH_TO_METER = 0.0254

# Joint-axis center-to-center lengths
L1_IN = 28.25
L2_IN = 16.00
L3_IN = 10.75

# Joint names must exactly match the URDF
JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
)

# IKPy includes the fixed base and fixed tool-tip joint
ACTIVE_LINKS_MASK = (
    False,  # Base link
    True,   # base_joint
    True,   # shoulder_joint
    True,   # elbow_joint
    True,   # wrist_joint
    False,  # tool_tip_joint
)

# URDF home-position tool coordinates
HOME_TARGET_IN = (
    0.0,
    L2_IN,
    21.5
)

# Viewer settings
MAX_IK_ERROR_IN = 0.05
ANIMATION_SMOOTHING = 0.15