# Milestone 5

In this milestone, you will learn how to use a voice recognition neural network, Whisper, and Text to Speech system, Espeak, and run LlaMa Large Language Model by Meta on Nvidia Jetson Xavier NX. 

## Helpful Guides

### Foxy
- [Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Building Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Publisher/Subscriber](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Development Environment

### Setup

1. Clone the repository
```sh
git clone https://github.com/CS7389K/Milestone-5.git
cd Milestone-5
```

2. ROS2 Foxy requires Ubuntu 20.04, so ensure it's what you're using. If you're using windows, run the following to use WSL:
```sh
install-wsl2-ros2-env.bat
```

3. Install Foxy
```sh
sh install-ros2-foxy-desktop.sh
```

### Building the Project

```sh
sh build.sh
. install/setup.sh
```
