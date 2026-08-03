from pathlib import Path
from threading import Lock

import sounddevice as sd
from piper import PiperVoice


# This assumes:
#
# RoboticArm/
# ├── Voice.py
# └── voices/
#     ├── en_GB-alan-medium.onnx
#     └── en_GB-alan-medium.onnx.json

VOICE_MODEL = (
    Path(__file__).resolve().parent
    / "voices"
    / "en_GB-alan-medium.onnx"
)

_speech_lock = Lock()

print("[voice] Loading Piper voice...")

if not VOICE_MODEL.exists():
    raise FileNotFoundError(
        f"Piper model was not found:\n{VOICE_MODEL}\n\n"
        "Make sure the .onnx and .onnx.json files are inside the voices folder."
    )

# Loaded once when Voice.py is first imported.
_voice = PiperVoice.load(str(VOICE_MODEL))

print("[voice] Piper voice ready.")


def speak(text: str) -> None:
    """
    Generate and play speech.

    Piper remains loaded in memory, so later responses begin much faster.
    This function blocks until the sentence finishes playing.
    """

    text = str(text).strip()

    if not text:
        return

    with _speech_lock:
        output_stream = None

        try:
            # Piper returns audio in chunks, allowing playback to begin
            # before the entire sentence has been generated.
            for chunk in _voice.synthesize(text):
                if output_stream is None:
                    output_stream = sd.RawOutputStream(
                        samplerate=chunk.sample_rate,
                        channels=chunk.sample_channels,
                        dtype="int16",
                    )
                    output_stream.start()

                output_stream.write(chunk.audio_int16_bytes)

        except Exception as error:
            print(f"[voice] Speech failed: {error}")

        finally:
            if output_stream is not None:
                output_stream.stop()
                output_stream.close()