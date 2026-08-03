from robot_control import RobotController


robot = RobotController()

try:
    robot.connect()

    print("Robot connected.")
    print("Commands: forward, backward, left, right, up, down, quit")

    while True:
        direction = input("\nDirection: ").strip().lower()

        if direction == "quit":
            break

        if direction not in {
            "forward",
            "backward",
            "left",
            "right",
            "up",
            "down",
        }:
            print("Unknown direction.")
            continue

        try:
            distance = float(
                input("Distance in inches: ")
            )

            if distance <= 0:
                print("Distance must be greater than zero.")
                continue

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
            print("Invalid movement:", error)

        except RuntimeError as error:
            print("Robot error:", error)

finally:
    robot.disconnect()