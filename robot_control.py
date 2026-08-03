import math

from config import (
    HOME_TARGET_IN,
    MAX_IK_ERROR_IN,
    SERIAL_BAUD,
    SERIAL_PORT,
    SERIAL_TIMEOUT,
)

from IK import (
    angles_by_name,
    forward_position_inches,
    move_forward,
    solve_ik,
)

from serial_control import ArduinoConnection


class RobotController:

    def __init__(self):
        """
        Start the software model in the URDF home pose.

        This assumes the physical robot is also placed and zeroed
        in exactly that pose before movement begins.
        """

        self.current_solution, error = solve_ik(
            HOME_TARGET_IN
        )

        if error > MAX_IK_ERROR_IN:
            raise RuntimeError(
                f"Could not initialize the home pose. "
                f"IK error: {error:.4f} inches"
            )

        self.current_position = list(HOME_TARGET_IN)

        self.arduino = ArduinoConnection(
            port=SERIAL_PORT,
            baud_rate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
        )


    def connect(self):
        """
        Connect to the Arduino.
        """
        self.arduino.connect()


    def disconnect(self):
        """
        Disconnect from the Arduino.
        """
        self.arduino.disconnect()


    def send_solution(self, solution):
        """
        Convert an IKPy solution from radians to degrees and
        send the four absolute joint angles to the Arduino.
        """

        angles = angles_by_name(solution)

        base_degrees = math.degrees(
            angles["base_joint"]
        )

        shoulder_degrees = math.degrees(
            angles["shoulder_joint"]
        )

        elbow_degrees = math.degrees(
            angles["elbow_joint"]
        )

        wrist_degrees = math.degrees(
            angles["wrist_joint"]
        )

        self.arduino.move_and_wait(
            base_degrees,
            shoulder_degrees,
            elbow_degrees,
            wrist_degrees,
        )


    def accept_movement(
        self,
        target,
        solution,
        error,
    ):
        """
        Check an IK solution, send it to the Arduino, and update
        the stored software pose after movement finishes.
        """

        if error > MAX_IK_ERROR_IN:
            raise ValueError(
                f"Target could not be reached accurately. "
                f"IK error: {error:.4f} inches"
            )

        self.send_solution(solution)

        # Only update the stored state after the Arduino says DONE.
        self.current_solution = solution
        self.current_position = list(target)

        print(
            "Movement completed. Current tool position:",
            self.current_position,
        )


    def move_forward(self, distance_inches):
        """
        Move outward in the direction the base currently faces.

        The base joint remains locked.
        """

        target, solution, error = move_forward(
            distance_inches,
            self.current_solution,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )


    def move_backward(self, distance_inches):
        """
        Move inward while keeping the base fixed.
        """

        self.move_forward(-distance_inches)


    def move_up(self, distance_inches):
        """
        Move vertically upward while preserving X and Y.
        """

        target = [
            self.current_position[0],
            self.current_position[1],
            self.current_position[2] + distance_inches,
        ]

        solution, error = solve_ik(
            target,
            self.current_solution,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )


    def move_down(self, distance_inches):
        """
        Move vertically downward while preserving X and Y.
        """

        self.move_up(-distance_inches)


    def move_to(self, x, y, z):
        """
        Move the tool tip to an absolute URDF XYZ coordinate.
        """

        target = [x, y, z]

        solution, error = solve_ik(
            target,
            self.current_solution,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )


    def get_position(self):
        """
        Return the software's current XYZ tool position.
        """

        return self.current_position.copy()


    def get_joint_angles_degrees(self):
        """
        Return the current URDF joint angles in degrees.
        """

        angles = angles_by_name(
            self.current_solution
        )

        return {
            joint_name: math.degrees(angle)
            for joint_name, angle in angles.items()
        }