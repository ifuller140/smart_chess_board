# Smart Chess Board - ROS 2 System

This repository contains the ROS 2 software stack for the Smart Chess Board project.

## Hardware Requirements
*   Raspberry Pi 4B (or newer)
*   2x Stepper Motors (28BYJ-48 + ULN2003)
*   1x Servo Motor (SG90 or similar)
*   3x Limit Switches
*   Raspberry Pi Camera Module (or USB Webcam)
*   CoreXY Gantry Frame

## Software Dependencies
*   Ubuntu 22.04 (Jammy)
*   ROS 2 Humble (or Iron)
*   Python 3 packages: `rpi.gpio`, `opencv-python`, `python-chess`

### Installation on Raspberry Pi

1.  **Install ROS 2 Humble**: Follow official instructions.
2.  **Install System Dependencies**:
    ```bash
    sudo apt update
    sudo apt install python3-pip python3-opencv
    pip3 install python-chess RPi.GPIO
    ```
3.  **Clone & Build**:
    ```bash
    mkdir -p ~/smart_chess_ws/src
    cd ~/smart_chess_ws/src
    # Clone this repo here
    cd ~/smart_chess_ws
    colcon build
    source install/setup.bash
    ```

## Configuration
Edit `src/chess_hw_interface/config/pins.yaml` to match your exact wiring.

## Running the System

1.  **Start Everything**:
    ```bash
    ros2 launch src/launch/full_system_launch.py
    ```

2.  **Homing**:
    The system will auto-home if configured, or run manually:
    ```bash
    ros2 run gantry_control homing_node
    ```

3.  **Start Game**:
    Press the clock button to trigger the first move detection.

## Troubleshooting
*   **Camera**: Check connection with `libcamera-hello`. If using USB, check `/dev/video0`.
*   **Motors**: Verify 5V power supply is separate from Pi logic power.
*   **Permissions**: Ensure user is in `dialout` and `gpio` groups.
