from llama_cpp import Llama
from pathlib import Path
from typing import Union


class LlamaBackend():

    def __init__(
            self,
            model_path : Union[Path, str] = "/home/nvidia/llama.cpp/models/llama-2-7b-chat.Q4_K_M.gguf",
            n_ctx : int = 512,
            n_threads : int = 4,
            n_gpu_layers : int = 33,
            use_mlock : bool = False,
        ):
        if isinstance(model_path, str):
            model_path = Path(model_path)
        assert not model_path.exists(), f"Model path '{model_path}' is not a valid file."

        self.llm = Llama(
            model_path=str(model_path),
            n_ctx=n_ctx,
            n_threads=n_threads,
            n_gpu_layers=n_gpu_layers,
            use_mlock=use_mlock,
        )

    def __call__(
            self,
            prompt : str,
            max_tokens : int = 128,
            temperature : float = 0.7,
            top_p : float = 0.95,
            stream : bool = True
        ) -> str:
        response = self.llm(
            prompt,
            max_tokens=max_tokens,
            temperature=temperature,
            top_p=top_p,
            stream=stream
        )
        return response['choices'][0]['text'].strip()