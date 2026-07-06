import time, re
import serial
import speech_recognition as sr
from JarvisBackend import speak, transcribe_once, get_input
import math
from old.IK import *

WAKE_WORD = "jarvis"
SERIAL_PORT = "/dev/tty.usbmodem1101"
BAUD = 115200

L1, L2, L3 = 28.5, 16, 11  # example link lengths in inches
Z_OFFSET = 0  # base height of shoulder from ground in inches
_start = find_angles(L1, L2, L3, L2, 0.0, Z_OFFSET + (L1 - L3))
current_angles = map_angles(*_start)
current_x, current_y, current_z = L2, 0.0, Z_OFFSET + (L1 - L3)

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

def handle_robot_command(text: str, current_x, current_y, current_z, current_angles):
    words = (text or "").lower().strip().split()
    print(f"[command] {words}")

    if "move" in words:
        unit = next((w for w in words if w in ("inches", "inch", "in")), None)
        if unit is None:
            speak("I didn't catch the distance.")
            return current_x, current_y, current_z, current_angles
        try:
            dist = float(words[words.index(unit) - 1])
        except (ValueError, IndexError):
            speak("I didn't catch the distance.")
            return current_x, current_y, current_z, current_angles
        
        base_rad = math.atan2(current_y, current_x)
        
        if "forward" in words or "forwards" in words:
            new_x = current_x - dist * math.cos(base_rad)
            new_y = current_y - dist * math.sin(base_rad)
            new_z = current_z
            
        elif "backward" in words or "backwards" in words:
            new_x = current_x + dist * math.cos(base_rad)
            new_y = current_y + dist * math.sin(base_rad)
            new_z = current_z
            
        elif "right" in words:
            new_x = current_x + dist * math.cos(base_rad - math.pi/2)
            new_y = current_y + dist * math.sin(base_rad - math.pi/2)
            new_z = current_z

        elif "left" in words:
            new_x = current_x + dist * math.cos(base_rad + math.pi/2)
            new_y = current_y + dist * math.sin(base_rad + math.pi/2)
            new_z = current_z
            
        elif "down" in words:
            new_x = current_x
            new_y = current_y
            new_z = current_z - dist
        
        elif "up" in words:
            new_x = current_x
            new_y = current_y
            new_z = current_z + dist
        
        else:
            speak("I didn't catch the direction.")
            return current_x, current_y, current_z, current_angles

        angles = find_angles(L1, L2, L3, new_x, new_y, new_z, z_offset=Z_OFFSET)
        if angles is None:
            speak("That position is out of reach.")
            return current_x, current_y, current_z, current_angles
        base_rad, shoulder_rad, elbow_rad, wrist_rad = angles
        base_deg, shoulder_deg, elbow_deg, wrist_deg = map_angles(base_rad, shoulder_rad, elbow_rad, wrist_rad)
        j0 = travel_distance(current_angles[0], base_deg)
        j1 = travel_distance(current_angles[1], shoulder_deg)
        j2 = travel_distance(current_angles[2], elbow_deg)
        j3 = travel_distance(current_angles[3], wrist_deg)

        speak(f"Moving {dist} inches {words[words.index(unit) + 1]}.")
        send_cmd(f"{j0},{j1},{j2},{j3}")
        print(f"[angles] base={base_deg:.1f}, shoulder={shoulder_deg:.1f}, elbow={elbow_deg:.1f}, wrist={wrist_deg:.1f}")
        print(f"[position] x={new_x:.2f}, y={new_y:.2f}, z={new_z:.2f}")
        
        return new_x, new_y, new_z, (base_deg, shoulder_deg, elbow_deg, wrist_deg)
    
    else:
        speak("Not sure what you mean.")
        return current_x, current_y, current_z, current_angles
    

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
                    current_x, current_y, current_z, current_angles = handle_robot_command(command, current_x, current_y, current_z, current_angles)
                print("Back to wake mode.")