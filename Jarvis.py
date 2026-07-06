import time, re
import serial
import speech_recognition as sr
from JarvisBackend import speak, transcribe_once, get_input
import math
from IK import *

WAKE_WORD = "jarvis"
SERIAL_PORT = "/dev/tty.usbmodem1101"
BAUD = 115200
ser = None

L1, L2, L3 = 28.5, 16, 11  # example link lengths in inches
Z_OFFSET = 0  # base height of shoulder from ground in inches
baseTheta, shoulderTheta, elbowTheta, wristTheta = 0, 0, 0, 0

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

def move(dx, dy, dz):
    
    global baseTheta, shoulderTheta, elbowTheta, wristTheta
    
    xCur, yCur, zCur = forward(baseTheta, shoulderTheta, elbowTheta, wristTheta, L1, L2, L3)
    xTarg = dx + xCur
    yTarg = dy + yCur
    zTarg = dz + zCur
    
    newBase, newShoulder, newElbow, newWrist = inverse(xTarg, yTarg, zTarg, L1, L2, L3)
    newBaseDeg = math.degrees(newBase - baseTheta)
    newShoulderDeg = math.degrees(newShoulder - shoulderTheta)
    newElbowDeg = math.degrees(newElbow - elbowTheta)
    newWristDeg = math.degrees(newWrist - wristTheta)
    
    baseTheta = newBase
    shoulderTheta = newShoulder
    elbowTheta = newElbow
    wristTheta = newWrist
    
    command = f"{newBaseDeg:.2f},{newShoulderDeg:.2f},{newElbowDeg:.2f},{newWristDeg:.2f}\n"
    ser.write(command.encode())
    
    return newBaseDeg, newShoulderDeg, newElbowDeg, newWristDeg
    