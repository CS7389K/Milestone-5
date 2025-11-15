import sys
import select
import subprocess

import whisper
import sounddevice as sd

from scipy.io.wavfile import write
from threading import Thread
from queue import Queue
from typing import Union


class WhisperWrapper():

    _tts_queue = Queue()

    def __init__(
            self,
            model_type : str = "small",
            device : str = "cpu",
        ):
        # Initialize Whisper model
        self.whisper = whisper.load_model(model_type, device=device)
        # Create TTS worker thread
        self._tts_thread = Thread(target=self._create_tts_worker, daemon=True)
        self._tts_thread.start()

    def _create_tts_worker(self) -> None:
        while True:
            text = self._tts_queue.get()
            if text is None:
                break
            # This blocks until this sentence is done speaking
            subprocess.run(
                ["espeak", "-s", "140", text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self._tts_queue.task_done()

    def speak(self, text: str) -> None:
        self._tts_queue.put(text)

    def __call__(
            self,
            duration_seconds : int = 5,
            samplerate : int = 16000,
            file_name : str = "tmp.wav"
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
        # Record audio ane save as file
        audio = sd.rec(
            int(duration_seconds * samplerate),
            samplerate=samplerate,
            channels=1,
            dtype="int16"
        )
        sd.wait()
        write(file_name, samplerate, audio)
        # Get text from file
        res = self.whisper.transcribe(
            file_name,
            temperature=0.0
        )
        return res["text"].strip()