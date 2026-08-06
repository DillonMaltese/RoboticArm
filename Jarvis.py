import re
import time

from JarvisBackend import (
    parse,
    record,
    shortRecord,
    transcribe,
)

from robot_control import RobotController
from Voice import speak


# -------------------- Voice configuration --------------------

WAKE_WORD = "jarvis"


def words_in(
    text: str,
) -> set[str]:
    """
    Return the lowercase words contained in transcribed text.

    Punctuation is ignored, so these all work:

        Jarvis
        Hey, Jarvis
        Okay Jarvis
    """

    return set(
        re.findall(
            r"[a-z]+",
            text.lower(),
        )
    )


def format_amount(
    amount: float,
) -> str:
    """
    Format a number naturally for speech.

    Examples:

        3.0 becomes "3"
        3.2 remains "3.2"
    """

    amount = float(amount)

    if amount.is_integer():
        return str(int(amount))

    return str(amount)


# -------------------- Wake-word handling --------------------

def wait_for_wake_word():
    """
    Listen repeatedly until the word Jarvis is detected.
    """

    print(
        "\nSay 'Hey Jarvis' to activate."
    )

    while True:
        audio = shortRecord()
        text = transcribe(audio)

        print(
            f"[wake] Heard: '{text}'"
        )

        if WAKE_WORD in words_in(text):
            return


def listen_for_command() -> str:
    """
    Say yes, then record and transcribe one command.
    """

    speak("Yes?")

    # Prevent the microphone from capturing the end of
    # Jarvis's spoken response.
    time.sleep(0.35)

    print(
        "Listening for command..."
    )

    audio = record()
    text = transcribe(audio)

    print(
        f"[command] Heard: '{text}'"
    )

    return text


# -------------------- Command execution --------------------

def execute_move_command(
    robot: RobotController,
    direction: str,
    amount: float,
):
    """
    Execute one directional movement.
    """

    amount = float(amount)

    if amount <= 0:
        speak(
            "The movement distance must be greater than zero."
        )
        return

    movement_functions = {
        "forward": robot.move_forward,
        "backward": robot.move_backward,
        "left": robot.move_left,
        "right": robot.move_right,
        "up": robot.move_up,
        "down": robot.move_down,
    }

    movement_function = movement_functions.get(
        direction
    )

    if movement_function is None:
        print(
            f"Unknown direction: {direction}"
        )

        speak(
            "I do not recognize that direction."
        )

        return

    spoken_amount = format_amount(
        amount
    )

    print(
        f"Moving {spoken_amount} inches {direction}."
    )

    speak(
        f"Moving {spoken_amount} inches {direction}."
    )

    movement_function(
        amount
    )

    speak("Done.")


def execute_command(
    robot: RobotController,
    text: str,
) -> bool:
    """
    Parse and execute one spoken command.

    Returns True to keep listening.
    Returns False to close voice control.
    """

    text = text.strip().lower()

    if not text:
        print("No command was heard.")
        speak("I did not hear a command.")
        return True

    command_words = words_in(text)

    # Exit voice control.
    if (
        "exit" in command_words
        or "quit" in command_words
        or "shutdown" in command_words
    ):
        print("Exiting voice control.")
        speak("Shutting down voice control.")
        return False

    command = parse(text)

    print(f"[command] Parsed: {command}")

    if command is None:
        speak("I did not understand that command.")
        return True

    command_type = command.get("type")


    # -------------------- Wrist-only movement --------------------

    if command_type == "wrist_rotate":
        degrees = float(
            command["degrees"]
        )

        direction = (
            "clockwise"
            if degrees > 0
            else "counterclockwise"
        )

        spoken_amount = format_amount(
            abs(degrees)
        )

        print(
            f"Calling robot.rotate_wrist({degrees})"
        )

        speak(
            f"Rotating wrist {spoken_amount} "
            f"degrees {direction}."
        )

        robot.rotate_wrist(
            degrees
        )

        speak("Done.")

        return True


    # -------------------- XYZ movement --------------------

    if command_type == "move":
        print(
            "Calling normal movement:",
            command["direction"],
            command["amount"],
        )

        execute_move_command(
            robot=robot,
            direction=command["direction"],
            amount=command["amount"],
        )

        return True


    # -------------------- Reset --------------------

    if command_type == "preset":
        preset_name = command.get("name")

        if preset_name == "reset":
            print("Calling robot.reset_to_start()")

            speak("Resetting.")

            robot.reset_to_start()

            speak("Reset complete.")

            return True

        if preset_name == "stop":
            speak(
                "No movement command is currently active."
            )

            return True


    print(
        f"Unknown command type: {command_type}"
    )

    speak(
        "I did not understand that command."
    )

    return True


# -------------------- Main program --------------------

def run():
    """
    Connect to the robot and start voice control.
    """

    robot = RobotController()
    connected = False

    try:
        robot.connect()
        connected = True

        print("Robot connected.")
        print()
        print(
            "Say 'Hey Jarvis', wait for 'Yes?', then say:"
        )
        print("  Move 3 inches forward")
        print("  Move 3.2 inches right")
        print("  Rotate wrist 45 degrees clockwise")
        print("  Rotate wrist 20 degrees counterclockwise")
        print("  Reset")
        print("  Exit")

        speak("Voice control ready.")

        keep_running = True

        while keep_running:
            wait_for_wake_word()

            command_text = listen_for_command()

            print(
                f"Passing command to execute_command: "
                f"'{command_text}'"
            )

            try:
                keep_running = execute_command(
                    robot,
                    command_text,
                )

            except ValueError as error:
                print(
                    f"Invalid movement: {error}"
                )

                speak(
                    "That movement is not valid."
                )

            except TimeoutError as error:
                print(
                    f"Arduino timeout: {error}"
                )

                speak(
                    "The robot did not finish responding."
                )

            except RuntimeError as error:
                print(
                    f"Robot error: {error}"
                )

                speak(
                    "I could not complete that command."
                )

            except Exception as error:
                print(
                    f"Unexpected command error: "
                    f"{type(error).__name__}: {error}"
                )

                speak(
                    "An unexpected error occurred."
                )

    except KeyboardInterrupt:
        print(
            "\nVoice control stopped."
        )

    finally:
        if connected:
            robot.disconnect()


if __name__ == "__main__":
    run()