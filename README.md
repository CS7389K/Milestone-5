# Milestone 5

In this milestone, you will learn how to use a voice recognition neural network, Whisper, and Text to Speech system, Espeak, and run LlaMa Large Language Model by Meta on Nvidia Jetson Xavier NX. 

## Helpful Guides

### Foxy
- [Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Action Server/Client](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

## Development Environment

### Setup

1. Clone the repository
```sh
git clone https://github.com/CS7389K/Milestone-5.git
cd Milestone-5
```

2. ROS2 Foxy requires Ubuntu 20.04, so ensure it's what you're using. If you're using windows, run the following to use WSL:
```sh
wsl --install -d Ubuntu-20.04
```

If you need to move WSL to a different drive (e.g. the F drive):
```
wsl --manage Ubuntu-20.04 --move F:\WSL
```

3. Install Foxy
```sh
sh install-ros2-foxy-desktop.sh
```

### Building the Project

Ensure you are in the project's root directory, then run:

```sh
colcon build --symlink-install
. install/setup.sh
```

### Running the Client / Server Nodes

#### Server
Using ESpeak and Whisper only:
```sh
ros2 run milestone5 server --ros-args -p use_espeak:=true use_whisper:=true -p
```

Using Llama's instruct model (for chat model, set `llama_instruct:=false`)
```sh
ros2 run milestone5 server --ros-args -p use_espeak:=true use_llama:=true -p llama_instruct:=true
```

##### Notes
- It is not advised to use both Whisper and Llama due to GPU, CPU, and RAM limitations
- If `llama_model_path` is specified, it will use that. If not, the system will use the following from `LlamaBackend` based on whether `llama_instruct` is `true` or `false`:

    ```python
    _MODEL_CHAT_PATH = "/home/nvidia/llama.cpp/models/llama-2-7b-chat.Q4_K_M.gguf"
    _MODEL_INSTRUCT_PATH = "/home/nvidia/llama.cpp/models/llama-2-7b-instruct-q4_0.gguf"
    ```

#### Client
```sh
ros2 run milestone5 client --ros-args -p use_espeak:=true use_llama:=false -p use_whisper:=true
```