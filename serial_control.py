import time
import serial


class ArduinoConnection:

    def __init__(
        self,
        port,
        baud_rate=115200,
        timeout=1,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None


    def connect(self):
        """
        Open the serial connection and wait for the Arduino to reset.
        """

        if self.serial is not None and self.serial.is_open:
            return

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
        )

        # Arduino boards normally reset when serial is opened.
        time.sleep(2)

        # Remove any startup messages left in the serial buffer.
        self.serial.reset_input_buffer()

        print(f"Connected to Arduino on {self.port}")


    def disconnect(self):
        """
        Close the serial connection.
        """

        if self.serial is not None and self.serial.is_open:
            self.serial.close()
            print("Arduino disconnected")


    def send_joint_angles(
        self,
        base_degrees,
        shoulder_degrees,
        elbow_degrees,
        wrist_degrees,
    ):
        """
        Send four absolute URDF joint angles to the Arduino.

        Message format:
            MOVE|base|shoulder|elbow|wrist
        """

        if self.serial is None or not self.serial.is_open:
            raise RuntimeError(
                "Arduino is not connected"
            )

        command = (
            f"MOVE|"
            f"{base_degrees:.6f}|"
            f"{shoulder_degrees:.6f}|"
            f"{elbow_degrees:.6f}|"
            f"{wrist_degrees:.6f}\n"
        )

        print("Sending:", command.strip())

        self.serial.reset_input_buffer()
        self.serial.write(command.encode("utf-8"))
        self.serial.flush()


    def wait_until_done(self, timeout_seconds=120):
        """
        Wait for DONE or ERROR from the Arduino.
        """

        if self.serial is None or not self.serial.is_open:
            raise RuntimeError(
                "Arduino is not connected"
            )

        start_time = time.time()

        while time.time() - start_time < timeout_seconds:

            response = (
                self.serial.readline()
                .decode("utf-8", errors="replace")
                .strip()
            )

            if not response:
                continue

            print("Arduino:", response)

            if response == "DONE":
                return

            if response.startswith("ERROR"):
                raise RuntimeError(
                    f"Arduino reported: {response}"
                )

        raise TimeoutError(
            "Arduino did not report DONE before the timeout"
        )


    def move_and_wait(
        self,
        base_degrees,
        shoulder_degrees,
        elbow_degrees,
        wrist_degrees,
    ):
        """
        Send one movement command and wait for completion.
        """

        self.send_joint_angles(
            base_degrees,
            shoulder_degrees,
            elbow_degrees,
            wrist_degrees,
        )

        self.wait_until_done()

    def reset_and_wait(self):
        """
        Send RESET to the Arduino and wait until it reports DONE.
        """

        if self.serial is None or not self.serial.is_open:
            raise RuntimeError(
                "Arduino is not connected."
            )

        print("Sending: RESET")

        self.serial.reset_input_buffer()

        self.serial.write(
            b"RESET\n"
        )

        self.serial.flush()

        self.wait_until_done()