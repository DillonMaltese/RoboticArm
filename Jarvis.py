import os, time, re
import serial
import speech_recognition as sr
from JarvisBackend import speak, transcribe_once, get_input
import math
from IK_DEMO import plot_robotic_arm

WAKE_WORD = "jarvis"
SERIAL_PORT = "/dev/tty.usbmodem1101"
BAUD = 115200

L1, L2, L3 = 10, 7.5, 2.5  # example link lengths in inches
Z_OFFSET = 0  # base height of shoulder from ground in inches

USE_MAPPED_ANGLES = True  # whether to use mapped angles for servo commands

ser = None

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

def send_cmd(cmd: str, expect_ok=True, speak_ok=False, max_wait_s=8.0):
    """Send a line to Arduino and wait for OK/ERR (or until timeout)."""
    try:
        s = open_serial()
        if not s:
            speak("I couldn't connect to the robot controller.")
            return
        # Creating and sending a command to Arduino
        line = (cmd.strip() + "\n").encode("ascii")
        s.write(line); s.flush()

        # Keep reading lines until OK/ERR or timeout
        start = time.time()
        reply = ""
        while time.time() - start < max_wait_s:
            r = s.readline().decode("utf-8", errors="ignore").strip()
            if not r:
                continue
            print(f"[arduino] {r}")
            reply = r
            if r.startswith(("OK", "ERR")):
                break

        if speak_ok:
            speak(reply if reply else "Command sent")
        if expect_ok and reply and not reply.startswith("OK"):
            print(f"[arduino] Unexpected reply: {reply!r}")
    except Exception as e:
        print("Serial error:", e)
        speak("I couldn't reach the robot controller.")

def handle_robot_command(text: str):
    t = (text or "").lower().strip()
    print(f"[command] {t}")

    # say hello / wave
    if "say hello" in t or "wave" in t or t == "hello":
        send_cmd("HELLO")
        speak("Hello, my name is Jarvis. Nice to meet you.")
        return

    # move X inches forward/backward
    m = re.search(r"(go|move)\s+to\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)", t)
    if m:
        X = float(m.group(2)); Y = float(m.group(3)); Z = float(m.group(4))

        angles = ik_planar_point_down(X, Y, Z)
        if angles is None:
            speak("That point is out of reach.")
            return

        base_rad, shoulder_rad, elbow_rad, wrist_rad = angles

        # For now: command shoulder/elbow/wrist
        j0_deg = math.degrees(base_rad)
        j1_deg = math.degrees(shoulder_rad)
        j2_deg = math.degrees(elbow_rad)
        j3_deg = math.degrees(wrist_rad)

        speak(f"Moving to {X} {Y} {Z}.")
        send_joint_targets_deg4(j0_deg, j1_deg, j2_deg, j3_deg)
        return

    # legacy fallbacks
    if "move forward" in t:
        speak("Say the distance, like 'move 5 inches forward'.")
        return
    if "move backward" in t:
        speak("Say the distance, like 'move 5 inches backward'.")
        return

    speak("Not sure what you're trying to say.")

def ik_planar_point_down(X, Y, Z):
    """
    Uses IK_DEMO.plot_robotic_arm().

    Returns:
      (base_rad, shoulder_rad, elbow_rad, wrist_rad)  # either raw or mapped depending on USE_MAPPED_ANGLES
    """
    global L1, L2, L3
    sol = plot_robotic_arm(L1, L2, L3, X, Y, Z, z_offset=Z_OFFSET)
    if sol is None:
        return None

    raw, mapped = sol
    return mapped if USE_MAPPED_ANGLES else raw

def send_joint_targets_deg4(j0_deg, j1_deg, j2_deg, j3_deg):
    send_cmd(f"JSET4 {j0_deg:.2f} {j1_deg:.2f} {j2_deg:.2f} {j3_deg:.2f}", expect_ok=True)

if __name__ == "__main__":
    rec = sr.Recognizer()
    rec.dynamic_energy_threshold = True
    rec.pause_threshold = 0.8

    with sr.Microphone() as source:
        print("Calibrating mic…")
        rec.adjust_for_ambient_noise(source, duration=1.0)
        print("Ready. Say 'Jarvis' to wake me.")

        while True:
            print("\nListening for wake word…")
            heard = transcribe_once(rec, source, limit=3)
            if not heard: continue
            print(f"[heard] {heard}")
            if WAKE_WORD in heard.lower():
                print("Wake word detected."); speak("Yes?")
                command = get_input(rec, source)
                if command:
                    handle_robot_command(command)
                print("Back to wake mode.")