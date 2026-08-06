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
    calculate_forward_move,
    forward_position_inches,
    solve_ik,
)

from serial_control import ArduinoConnection


class RobotController:
    def __init__(self):
        """
        Start the software model in the URDF home pose.

        The physical robot must also be placed in the home pose
        when the Arduino starts.
        """

        home_solution, error = solve_ik(
            HOME_TARGET_IN
        )

        if error > MAX_IK_ERROR_IN:
            raise RuntimeError(
                "Could not initialize the home pose. "
                f"IK error: {error:.4f} inches"
            )

        # Keep a permanent copy of the startup configuration.
        self.home_solution = home_solution.copy()

        # Current software state.
        self.current_solution = self.home_solution.copy()
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
        Send relative joint-angle changes to the Arduino.

        The Arduino uses AccelStepper.move(), so it expects the
        difference between the current and target joint angles.
        """

        current_angles = angles_by_name(
            self.current_solution
        )

        target_angles = angles_by_name(
            solution
        )

        base_change = math.degrees(
            target_angles["base_joint"]
            - current_angles["base_joint"]
        )

        shoulder_change = math.degrees(
            target_angles["shoulder_joint"]
            - current_angles["shoulder_joint"]
        )

        elbow_change = math.degrees(
            target_angles["elbow_joint"]
            - current_angles["elbow_joint"]
        )

        wrist_change = math.degrees(
            target_angles["wrist_joint"]
            - current_angles["wrist_joint"]
        )

        print("Relative joint changes:")
        print(f"  Base:     {base_change:.6f}")
        print(f"  Shoulder: {shoulder_change:.6f}")
        print(f"  Elbow:    {elbow_change:.6f}")
        print(f"  Wrist:    {wrist_change:.6f}")

        self.arduino.move_and_wait(
            base_change,
            shoulder_change,
            elbow_change,
            wrist_change,
        )


    def accept_movement(
        self,
        target,
        solution,
        error,
    ):
        """
        Validate an IK solution, move the physical robot, and then
        update the stored software position.
        """

        if error > MAX_IK_ERROR_IN:
            raise ValueError(
                "Target could not be reached accurately. "
                f"IK error: {error:.4f} inches"
            )

        self.send_solution(solution)

        # Update software only after the Arduino reports DONE.
        self.current_solution = solution.copy()
        self.current_position = [
            float(target[0]),
            float(target[1]),
            float(target[2]),
        ]

        print(
            "Movement completed. Current tool position:",
            self.current_position,
        )


    def move_forward(self, distance_inches):
        """
        Move outward while keeping the base angle fixed.
        """

        target, solution, error = calculate_forward_move(
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
        Move inward while keeping the base angle fixed.
        """

        self.move_forward(
            -distance_inches
        )


    def move_up(self, distance_inches):
        """
        Move vertically upward while preserving X and Y.
        """

        target = [
            float(self.current_position[0]),
            float(self.current_position[1]),
            float(self.current_position[2]) + distance_inches,
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

        self.move_up(
            -distance_inches
        )


    def move_right(self, distance_inches):
        """
        Move toward global +X while keeping Y and Z unchanged.
        """

        target = [
            float(self.current_position[0]) + distance_inches,
            float(self.current_position[1]),
            float(self.current_position[2]),
        ]

        starting_guess = self.current_solution.copy()

        # The arm's zero pose points along +Y.
        # Give IKPy an estimate of the required base rotation.
        starting_guess[1] = math.atan2(
            -target[0],
            target[1],
        )

        solution, error = solve_ik(
            target,
            starting_guess,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )


    def move_left(self, distance_inches):
        """
        Move toward global -X while keeping Y and Z unchanged.
        """

        target = [
            float(self.current_position[0]) - distance_inches,
            float(self.current_position[1]),
            float(self.current_position[2]),
        ]

        starting_guess = self.current_solution.copy()

        starting_guess[1] = math.atan2(
            -target[0],
            target[1],
        )

        solution, error = solve_ik(
            target,
            starting_guess,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )


    def move_to(self, x, y, z):
        """
        Move to an absolute URDF XYZ coordinate.
        """

        target = [
            float(x),
            float(y),
            float(z),
        ]

        starting_guess = self.current_solution.copy()

        # Supply a useful base-angle estimate whenever the target
        # is not directly above the base.
        if abs(target[0]) > 1e-9 or abs(target[1]) > 1e-9:
            starting_guess[1] = math.atan2(
                -target[0],
                target[1],
            )

        solution, error = solve_ik(
            target,
            starting_guess,
        )

        self.accept_movement(
            target,
            solution,
            error,
        )

    def rotate_wrist(self, degrees_clockwise):
        """
        Rotate only the wrist.

        Positive input means clockwise.
        Negative input means counterclockwise.
        """

        degrees_clockwise = float(
            degrees_clockwise
        )

        if degrees_clockwise == 0:
            raise ValueError(
                "Wrist rotation cannot be zero."
            )

        # Python joint signs are opposite the physical directions
        # established by the working Arduino tests.
        wrist_command = -degrees_clockwise

        print(
            "Sending wrist-only movement:"
        )
        print(
            f"  Requested physical rotation: "
            f"{degrees_clockwise:.6f} degrees"
        )
        print(
            f"  Wrist value sent to Arduino: "
            f"{wrist_command:.6f} degrees"
        )

        self.arduino.move_and_wait(
            0.0,
            0.0,
            0.0,
            wrist_command,
        )

        # Keep Python's wrist angle synchronized with the command.
        self.current_solution[4] += math.radians(
            wrist_command
        )

        print("Wrist movement completed.")


    def reset_to_start(self):
        """
        Return the motors to the step positions recorded when the
        Arduino started, then reset the Python model to home.
        """

        print(
            "Returning robot to startup position..."
        )

        # The Arduino returns every motor to currentPosition() == 0.
        self.arduino.reset_and_wait()

        # Update Python only after the Arduino reports DONE.
        self.current_solution = self.home_solution.copy()
        self.current_position = list(HOME_TARGET_IN)

        print(
            "Reset completed. Current tool position:",
            self.current_position,
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