import os, time, threading, re
import speech_recognition as sr
from playsound import playsound
from backend import speak, transcribe_once, get_input

WAKE_WORD = "jarvis"

# Where we determine what to do given a command
def handle_robot_command(text: str):
    if "arm right" in text:
        speak("Moving arm right")
        
    elif "arm left" in text:
        speak("Moving arm left")
        
    elif "arm up" in text:
        speak("Moving arm up")
        
    elif "arm down" in text:
        speak("Moving arm down")

    else:
        speak("not sure what you're trying to say")

def schedule_reminder(task: str, when_dt: datetime, speak):
    delay = max((when_dt - datetime.now(TZ)).total_seconds(), 0)
    def _ding():
        speak(f"Reminder: {task}")
    timer = threading.Timer(delay, _ding)
    timer.daemon = True
    timer.start()
    return timer

if __name__ == "__main__":
    rec = sr.Recognizer()
    rec.dynamic_energy_threshold = True
    rec.pause_threshold = 0.8

    with sr.Microphone() as source:
        print("Calibrating mic…"); rec.adjust_for_ambient_noise(source, duration=1.0)
        # speak("Jarvis Activated")
        speak("Good evening. All systems are nominal.")
        print("Ready. Say 'Jarvis' to wake me.")

        while True:
            print("\nListening for wake word…")
            heard = transcribe_once(rec, source, limit=3)
            if not heard: continue
            print(f"[heard] {heard}")
            if WAKE_WORD in heard:
                print("Wake word detected."); speak("Yes?")
                command = get_input(rec, source)
                if command:
                    handle_robot_command(command)
                print("Back to wake mode.")
