import time
import serial
from JarvisBackend import record, transcribe, parse, shortRecord
import math
from IK import *
from Voice import speak

WAKE_WORD = "jarvis"
SERIAL_PORT = "COM4"
BAUD = 115200
ser = None

baseTheta     = math.radians(0)
shoulderTheta = math.radians(30.95)
elbowTheta    = math.radians(90.0)
wristTheta    = math.radians(-210.95)


L1, L2, L3 = 28.25, 16, 10.75  # example link lengths in inches
Z_OFFSET = 0  # base height of shoulder from ground in inches

def open_serial():
    global ser
    if ser and ser.is_open:
        return ser
    try:
        # longer timeout to allow motion before reply
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=2.0)
        time.sleep(2.0)  # let Arduino reset
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"[serial] Connected to {SERIAL_PORT}")
        return ser
    except Exception as e:
        print("[serial] Could not open serial port:", e)
        return None

def move(dx_radial, d_base_angle, dz):
    global baseTheta, shoulderTheta, elbowTheta, wristTheta

    xCur, yCur, zCur = forward(baseTheta, shoulderTheta, elbowTheta, wristTheta, L1, L2, L3)

    # Move in the direction the arm faces (negative X)
    xTarg = xCur + dx_radial * math.cos(baseTheta)
    yTarg = yCur + dx_radial * math.sin(baseTheta)
    zTarg = zCur + dz

    if d_base_angle != 0:
        r = math.sqrt(xCur**2 + yCur**2)
        newBase = baseTheta + d_base_angle
        xTarg = r * math.cos(newBase)   # removed the negative
        yTarg = r * math.sin(newBase)
        zTarg = zCur

    # Flip X before passing to inverse since it expects positive X targets
    newBase, newShoulder, newElbow, newWrist = inverse(xTarg, yTarg, zTarg, L1, L2, L3)

    newBaseDeg     = math.degrees(newBase     - baseTheta)
    newShoulderDeg = math.degrees(newShoulder - shoulderTheta)
    newElbowDeg    = math.degrees(newElbow    - elbowTheta)
    newWristDeg    = math.degrees(newWrist    - wristTheta)

    baseTheta     = newBase
    shoulderTheta = newShoulder
    elbowTheta    = newElbow
    wristTheta    = newWrist

    command = f"|{newBaseDeg:.2f}|{-newShoulderDeg:.2f}|{-newElbowDeg:.2f}|{newWristDeg:.2f}|\n"
    if ser and ser.is_open:
        ser.write(command.encode())
    else:
        print(f"[serial] Would send: {command.strip()}")

    return newBaseDeg, newShoulderDeg, newElbowDeg, newWristDeg


def wakeListen():
    while True:
        audio = shortRecord()
        text  = transcribe(audio)
        print(f"[wake] Heard: '{text}'")
        if WAKE_WORD in text:
            speak("Yes?")
            return
    

def run():
    open_serial()
    print("Say Jarvis to activate...")
    speak("Say Jarvis to activate...")

    while True:

        wakeListen()
        print("Listening for command")

        audio = record()
        text  = transcribe(audio)

        if not text:
            continue

        print(f"Heard: '{text}'")
        textCommand = parse(text)

        if textCommand is None:
            print("Could not parse command, listening again...")
            continue

        print(f"Parsed: {textCommand}")

        if textCommand["type"] == "move":
            direction = textCommand["direction"]
            amount    = textCommand["amount"]

            if direction in ("forward", "backward", "back"):
                dx_radial    = amount if direction == "forward" else -amount
                d_base_angle = 0
                dz           = 0

            elif direction in ("left", "right"):
                dx_radial    = 0
                d_base_angle = math.radians(amount) if direction == "right" else -math.radians(amount)
                dz           = 0

            elif direction == "up":
                dx_radial    = 0
                d_base_angle = 0
                dz           = amount

            elif direction == "down":
                dx_radial    = 0
                d_base_angle = 0
                dz           = -amount

            else:
                print("Unknown direction")
                continue

            speak("Moving " + str(amount) + " " + direction)
            angles = move(dx_radial, d_base_angle, dz)
            print(f"Sent: base={angles[0]:.2f} shoulder={angles[1]:.2f} elbow={angles[2]:.2f} wrist={angles[3]:.2f}")

        elif textCommand["type"] == "preset":
            speak(f"Moving to {textCommand['name']} position.")
            print(f"Preset '{textCommand['name']}' not yet implemented")

if __name__ == "__main__":
    run()
