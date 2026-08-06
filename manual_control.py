from robot_control import RobotController


robot = RobotController()


try:
    robot.connect()

    print("Robot connected.")
    print(
        "Commands: forward, backward, left, right, "
        "up, down, reset, quit"
    )

    while True:
        direction = input(
            "\nDirection: "
        ).strip().lower()

        if direction == "quit":
            break

        if direction == "reset":
            robot.reset_to_start()
            continue

        valid_directions = {
            "forward",
            "backward",
            "left",
            "right",
            "up",
            "down",
        }

        if direction not in valid_directions:
            print("Unknown direction.")
            continue

        try:
            distance = float(
                input("Distance in inches: ")
            )
        except ValueError:
            print(
                "Distance must be a number."
            )
            continue

        if distance <= 0:
            print(
                "Distance must be greater than zero."
            )
            continue

        try:
            if direction == "forward":
                robot.move_forward(distance)

            elif direction == "backward":
                robot.move_backward(distance)

            elif direction == "left":
                robot.move_left(distance)

            elif direction == "right":
                robot.move_right(distance)

            elif direction == "up":
                robot.move_up(distance)

            elif direction == "down":
                robot.move_down(distance)

        except ValueError as error:
            print(
                f"Invalid movement: {error}"
            )

        except RuntimeError as error:
            print(
                f"Movement failed: {error}"
            )

        except TimeoutError as error:
            print(
                f"Arduino timeout: {error}"
            )


finally:
    robot.disconnect()