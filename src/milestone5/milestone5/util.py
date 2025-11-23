import sys
import select

import sounddevice as sd
from scipy.io.wavfile import write
from typing import Optional


def record_audio_and_saveas(
        file_name,
        duration_seconds : int = 5,
        sample_rate : int = 16000
    ):
    audio = sd.rec(
        int(duration_seconds * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype="int16"
    )
    sd.wait()
    write(file_name, sample_rate, audio)


def get_user_input(
        client,
        file_name : str = "voice_input.wav",
    ) -> Optional[str]:
    # Prompt user for input
    sys.stdout.write("\nYou (type or 's'→voice): ")
    sys.stdout.flush()
    # Get user input
    ready, _, _ = select.select([sys.stdin], [], [], 5)
    if ready:
        text = sys.stdin.readline().strip()
        if text.lower() in ("exit", "quit"):
            return None

        elif text.lower() == "s":
            client("espeak", "speak in voice now")
            # Record audio from microphone
            record_audio_and_saveas(file_name)
            # Transcribe audio file with whisper
            text = client("whisper", file_name)

        return text


def prompt_assistant(
        client,
        prompt : str
    ) -> Optional[str]:
    print("Assistant: ", end="", flush=True)
    buf = ""
    for chunk in client("llama", prompt):
        token = chunk["choices"][0]["text"]
        buf += token
        print(token, end="", flush=True)
        if token.endswith((" ", ".", "?", "!")):
            client("espeak", buf)
            buf = ""
    if buf.strip():
        client("espeak", buf)
    print()