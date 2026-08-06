from __future__ import annotations

import re

import numpy as np
import sounddevice as sd
import torch
import whisper


# -------------------- Whisper configuration --------------------

MODEL_NAME = "medium"

DEVICE = (
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)

USE_FP16 = DEVICE == "cuda"

print(
    f"Loading Whisper model '{MODEL_NAME}' "
    f"on {DEVICE}..."
)

model = whisper.load_model(
    MODEL_NAME,
    device=DEVICE,
)

print("Whisper model loaded.")


# -------------------- Recording configuration --------------------

SAMPLE_RATE = 16000

# Time allowed for a complete movement command.
DURATION = 4.0

# Time allowed for the wake phrase.
WAKE_DURATION = 2.0


def record_for_duration(
    duration_seconds: float,
) -> np.ndarray:
    """
    Record mono microphone audio and return a one-dimensional
    float32 NumPy array for Whisper.
    """

    number_of_samples = int(
        duration_seconds * SAMPLE_RATE
    )

    audio = sd.rec(
        number_of_samples,
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
    )

    sd.wait()

    return audio.flatten()


def shortRecord() -> np.ndarray:
    """
    Record a short audio segment for detecting the wake word.
    """

    return record_for_duration(
        WAKE_DURATION
    )


def record() -> np.ndarray:
    """
    Record a longer audio segment for a complete command.
    """

    return record_for_duration(
        DURATION
    )


# -------------------- Speech transcription --------------------

WHISPER_PROMPT = (
    "hey jarvis, yes, move forward, move backward, rotate, wrist"
    "move left, move right, move up, move down, "
    "inch, inches, reset, go home, stop, exit, "
    "one, two, three, four, five, six, seven, "
    "eight, nine, ten, half, point, decimal, "
    "move three point two inches right"
)


def transcribe(
    audio: np.ndarray,
) -> str:
    """
    Convert recorded audio into lowercase English text.
    """

    if audio is None or len(audio) == 0:
        return ""

    result = model.transcribe(
        audio,
        language="en",
        fp16=USE_FP16,
        initial_prompt=WHISPER_PROMPT,
        temperature=0.0,
        condition_on_previous_text=False,
    )

    return result["text"].strip().lower()


# -------------------- Text processing --------------------

TOKEN_PATTERN = re.compile(
    r"\d+(?:\.\d+)?|[a-z]+"
)


def tokenize(
    text: str,
) -> list[str]:
    """
    Split text into lowercase words and numeric values.

    Examples:

        "Move 3.2 inches right."
        becomes
        ["move", "3.2", "inches", "right"]
    """

    return TOKEN_PATTERN.findall(
        text.lower()
    )


# -------------------- Number parsing --------------------

WORD_TO_NUM = {
    "zero": 0,
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
    "six": 6,
    "seven": 7,
    "eight": 8,
    "nine": 9,
    "ten": 10,
    "eleven": 11,
    "twelve": 12,
    "thirteen": 13,
    "fourteen": 14,
    "fifteen": 15,
    "sixteen": 16,
    "seventeen": 17,
    "eighteen": 18,
    "nineteen": 19,
    "twenty": 20,
}


DIGIT_WORDS = {
    "zero": "0",
    "one": "1",
    "two": "2",
    "three": "3",
    "four": "4",
    "five": "5",
    "six": "6",
    "seven": "7",
    "eight": "8",
    "nine": "9",
}


DECIMAL_WORDS = {
    "point",
    "decimal",
    "dot",
}


def extract_decimal_digits(
    words: list[str],
    start_index: int,
) -> str:
    """
    Extract decimal digits beginning at start_index.

    Examples:

        ["two", "five"] becomes "25"
        ["5"] becomes "5"
    """

    digits = []

    for word in words[start_index:]:
        if word in DIGIT_WORDS:
            digits.append(
                DIGIT_WORDS[word]
            )

        elif word.isdigit():
            digits.extend(word)

        else:
            break

    return "".join(digits)


def extract_number(
    words: list[str],
) -> float | None:
    """
    Extract a movement amount from command tokens.

    Supported examples:

        3
        3.2
        three
        three point two
        point five
        half
        half an inch
        a half inch
    """

    for index, word in enumerate(words):

        # "a half inch" or "an half inch"
        if (
            word in {"a", "an"}
            and index + 1 < len(words)
            and words[index + 1] == "half"
        ):
            return 0.5

        if word == "half":
            return 0.5

        # Handle values beginning with the decimal word:
        # "point five"
        if word in DECIMAL_WORDS:
            decimal_digits = extract_decimal_digits(
                words,
                index + 1,
            )

            if decimal_digits:
                return float(
                    f"0.{decimal_digits}"
                )

            continue

        base_value = None

        # Numeric transcription, such as "3" or "3.2".
        try:
            base_value = float(word)
        except ValueError:
            pass

        # Written number, such as "three".
        if (
            base_value is None
            and word in WORD_TO_NUM
        ):
            base_value = float(
                WORD_TO_NUM[word]
            )

        if base_value is None:
            continue

        # Handle:
        # "three point two"
        # "3 point 2"
        if (
            index + 1 < len(words)
            and words[index + 1] in DECIMAL_WORDS
        ):
            decimal_digits = extract_decimal_digits(
                words,
                index + 2,
            )

            if decimal_digits:
                whole_part = int(base_value)

                return float(
                    f"{whole_part}.{decimal_digits}"
                )

        return float(base_value)

    return None


# -------------------- Command parsing --------------------

DIRECTION_ALIASES = {
    "forward": "forward",
    "forwards": "forward",

    "back": "backward",
    "backward": "backward",
    "backwards": "backward",

    "left": "left",
    "right": "right",

    "up": "up",
    "upward": "up",
    "upwards": "up",

    "down": "down",
    "downward": "down",
    "downwards": "down",
}


PRESET_ALIASES = {
    "reset": "reset",
    "home": "reset",
    "rest": "reset",
    "stop": "stop",
}


def parse(
    text: str,
) -> dict | None:
    """
    Parse a spoken command.

    Supported examples:

        Move 3 inches forward
        Move 3.2 inches right
        Move three point two inches left
        Rotate wrist 10 degrees clockwise
        Turn wrist 5 degrees counterclockwise
        Reset
        Go home
        Stop
    """

    words = tokenize(text)

    if not words:
        return None


    # -------------------- Wrist-only rotation --------------------

    wrist_command = (
        "wrist" in words
        and (
            "rotate" in words
            or "turn" in words
            or "move" in words
        )
    )

    if wrist_command:
        amount = extract_number(words)

        if amount is None:
            return None

        counterclockwise = (
            "counterclockwise" in words
            or "anticlockwise" in words
            or (
                "counter" in words
                and "clockwise" in words
            )
        )

        clockwise = (
            "clockwise" in words
            and not counterclockwise
        )

        if counterclockwise:
            signed_degrees = -float(amount)

        elif clockwise:
            signed_degrees = float(amount)

        else:
            # A wrist command must specify a direction.
            return None

        return {
            "type": "wrist_rotate",
            "degrees": signed_degrees,
        }


    # -------------------- Presets --------------------

    for word in words:
        if word in PRESET_ALIASES:
            return {
                "type": "preset",
                "name": PRESET_ALIASES[word],
            }


    # -------------------- Directional movement --------------------

    direction = None

    for word in words:
        if word in DIRECTION_ALIASES:
            direction = DIRECTION_ALIASES[word]
            break

    if direction is None:
        return None

    amount = extract_number(words)

    if amount is None:
        return None

    return {
        "type": "move",
        "direction": direction,
        "amount": float(amount),
    }