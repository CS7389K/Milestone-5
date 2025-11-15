import sys
import select

import whisper
import sounddevice as sd
from scipy.io.wavfile import write


class WhisperBackend():

    def __init__(
            self,
            model_type : str = "small",
            device : str = "cpu",
            duration_seconds : int = 5,
            sample_rate : int = 16000,
        ):
        # Initialize Whisper model
        self.whisper = whisper.load_model(model_type, device=device)
        self.duration_seconds = duration_seconds
        self.sample_rate = sample_rate

    def __call__(
            self,
            file_name: str = "tmp.wav"
        ) -> str:
        sys.stdout.write("\nYou (type or 's'/wait→voice): ")
        sys.stdout.flush()
        ready, _, _ = select.select([sys.stdin], [], [], 5)
        if ready:
            line = sys.stdin.readline().strip()
            if   line.lower() in ("exit","quit"):
                return None
            elif line.lower() == "s":
                self.speak("speak in voice now")
            else:
                return line
        # Record audio and save as file
        audio = sd.rec(
            int(self.duration_seconds * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype="int16"
        )
        sd.wait()
        write(file_name, self.sample_rate, audio)
        # Get text from file
        res = self.whisper.transcribe(
            file_name,
            temperature=0.0
        )
        return res["text"].strip()