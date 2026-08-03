import whisper
import sounddevice as sd
import numpy as np

model = whisper.load_model("medium", device="cuda")


SAMPLE_RATE = 16000
DURATION    = 3
WAKE_DURATION = 2

def shortRecord():
    audio = sd.rec(
        int(WAKE_DURATION * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32"
    )
    sd.wait()
    return audio.flatten()

def record():
    audio = sd.rec(
        int(DURATION * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32"
    )
    sd.wait()
    #Turning 2D array that sounddevice returns (shape: 64000 x 1) into a 1D array (shape: 64000) for whisper
    return audio.flatten()

#Making the most common words that whisper will hear known
WHISPER_PROMPT = (
    "hey jarvis move forward backward left right up down "
    "inch inches stop home rest"
)

def transcribe(audio: np.ndarray) -> str:
    result = model.transcribe(
        audio,
        language="en",
        fp16=True,
        #Telling whisper to expect the words from the variable above
        initial_prompt=WHISPER_PROMPT,
        #Making no randomness
        temperature=0.0,
        condition_on_previous_text=False
    )
    #Removes wbitespace + converts to lowercase
    return result["text"].strip().lower()

WORD_TO_NUM = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
    "half": 0.5, "a": 1, "an": 1
}
#Making sure there are numbers and no words
def extract_number(words: list[str]) -> float | None:
    for word in words:
        if word in WORD_TO_NUM:
            return float(WORD_TO_NUM[word])
        try:
            return float(word)
        except ValueError:
            continue
    return None

#Mapping everything to unit vector
DIRECTION_MAP = {
    "forward":  ( 1,  0,  0),   # dx_radial, d_base_angle, dz
    "backward": (-1,  0,  0),
    "back":     (-1,  0,  0),
    "left":     ( 0, -1,  0),   # negative base rotation = CCW
    "right":    ( 0,  1,  0),   # positive base rotation = CW
    "up":       ( 0,  0,  1),
    "down":     ( 0,  0, -1),
}

PRESETS = ["home", "rest", "stop"]

def parse(text: str) -> dict | None:
    words = text.strip().lower().split()

    # ── Preset / stop ──
    for preset in PRESETS:
        if preset in words:
            return {"type": "preset", "name": preset}

    # ── Directional move ──
    direction = None
    for word in words:
        if word in DIRECTION_MAP:
            direction = word
            break

    if direction is None:
        return None

    amount = extract_number(words)
    if amount is None:
        return None

    return {
        "type":      "move",
        "direction": direction,
        "amount":    amount,
        "vector":    tuple(v * amount for v in DIRECTION_MAP[direction])
    }