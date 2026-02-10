# Sea Puppy Low Level Control System V1.3
### Made by UVic Environmental Engineering Club ([UVEEC](https://uveec.ca/)) at [University of Victoria](https://www.uvic.ca/)
[![stmicroelectronics](https://img.shields.io/badge/stmicroelectronics-03234B.svg?&logo=stmicroelectronics)](https://www.st.com/content/st_com/en.html)
[![c](https://img.shields.io/badge/c-A8B9CC.svg?&logo=c)](https://en.cppreference.com/w/c/language)
[![cplusplus](https://img.shields.io/badge/c++-00599C.svg?&logo=cplusplus)](https://www.st.com/content/st_com/en.html)
[![ros](https://img.shields.io/badge/ros-22314E.svg?&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Description
This repo is part of Sea Glider project (SeaPuppy) by UVEEC. Sea glider is an unmanned underwater vehicle (UUV) for ocean research. Compare to conventional submarine, glider is more efficient because it utilizes buoyance and gavity as driving power. 

This is a low level control system for the sea glider written in C/C++. This repo is responsible for STM32 on UVEEC's PCB "Megamind". "Megamind" is responsible for reading sensors and actuating engines on the sea glider. Full documentation about Low Level Control System can be found [here](https://docs.google.com/document/d/1BvOxKdqG76WM1FazrVoIZYhodaNmmSBSupSmKFFQtNc/edit?tab=t.0) (only available for UVEEC member).

CUBE IDE development environment was used on Windows 10. MicroROS for ROS2 Humble has been deployed. MicroROS is to be used to communication with ROS2 Humble on Raspberry Pi with Blue OS from Blue Robotics. If you need access to this documentation, please create [issue]((https://github.com/UVic-Environmental-Engineering-Club/SeaPuppy1.3/issues)) on this project.

## Getting Started (If you are UVEEC member)
1. Go to uveec club room and log into the desktop.
2. Open STM32 Cube IDE.
3. Select SeaPuppy1.3 on Project Explorer.
4. main.c can be found at SeaPuppy1.3 -> Core -> Src -> main.c
5. Enjoy!

## Getting Started (If you are not UVEEC member)

### Prerequisites
- Install **V1.18.0** [STM32 CUBE IDE](https://www.st.com/en/development-tools/stm32cubeide.html) on Windows 10/11
- Install [Docker Desktop](https://www.docker.com/products/docker-desktop) and make sure it's running
- Git

### Setup Steps
1. **Clone the repository**:
   ```bash
   git clone https://github.com/UVic-Environmental-Engineering-Club/CETOLLCS1.3.git
   cd CETOLLCS1.3
   ```

2. **Build the micro-ROS library with custom interfaces**:
   
   > **IMPORTANT**: This step is **REQUIRED** before building the project. The custom ROS2 message interfaces are NOT included in the repository and must be compiled locally.
   
   Open a terminal (PowerShell or Command Prompt) in the project folder and run:
   ```bash
   docker pull microros/micro_ros_static_library_builder:humble
   docker run --rm -v <ABSOLUTE_PATH_TO_PROJECT>:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
   ```
   
   Replace `<ABSOLUTE_PATH_TO_PROJECT>` with your full project path. For example:
   - Windows: `C:\Users\YourName\CETOLLCS1.3`
   - The command would be: `docker run --rm -v C:\Users\YourName\CETOLLCS1.3:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble`
   
   This will generate the `libmicroros` library including the UVEEC custom interfaces (`uveec_custom_interfaces`).

3. **Import the project into STM32CubeIDE**:
   - Open STM32CubeIDE
   - File → Import → General → Existing Projects into Workspace
   - Select the cloned project directory
   - Click Finish

4. **Build the project**:
   - Click the Build button (hammer icon) in STM32CubeIDE
   - The project should now compile successfully

5. **Connect and Flash**:
   - Connect your STM32 with ST-Link
   - Connect your ST-Link to your Windows machine
   - Click Run to flash the firmware
   - (CubeIDE might prompt to install ST-Link-Server - follow the instructions)

6. **Deploy**:
   - You can now unplug your STM32
   - Plug in your STM32 to Raspberry Pi with Navigator Flight Controller
   - Follow instruction on Raspberry Pi documentation (To be developed and documented)

### Troubleshooting

**Build Error: "uveec_custom_interfaces/msg/raspberry_sensors_interface.h: No such file or directory"**
- You forgot to run the Docker command in step 2
- Or the Docker build failed - delete `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/` and re-run the Docker command

**Linker Error: "cannot find -lmicroros"**
- The Docker build didn't complete successfully
- Make sure Docker Desktop is running
- Delete `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/` and re-run the Docker command

**Docker says "micro-ROS library found. Skipping..."**
- An old/incomplete library exists
- Delete `micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/` to force a rebuild

## Relevent documentations
* [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
* [First micro-ROS Application on Linux](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
* [ROS2 humble](https://docs.ros.org/en/humble/index.html)
