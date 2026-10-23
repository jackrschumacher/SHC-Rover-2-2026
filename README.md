# SHC Rover 2

```
Laptop                        Orange Pi (on rover)              Pico
ground_station  --ROS 2-->    onboard  --USB serial (JSON)-->   pico/rover_pico
 (pygame)       <--ROS 2--             <--                      (servos, motor, sensors, SD)
```

| Path | What |
|---|---|
| `ros2_ws/src/rover_comms` | Messages: `ArmCommand` (laptop → rover), `Telemetry` (rover → laptop) |
| `ros2_ws/src/rover_pkg` | `ground_station` (pygame) and `onboard` (serial bridge) nodes |
| `pico/rover_pico` | Pico firmware (Arduino, Earle Philhower RP2040 core). Pins and limits in `config.h` |
| `tools/pico_link_check.py` | Bench test: read Pico telemetry over serial without ROS |

## Build (laptop and Orange Pi)

```bash
sudo apt install python3-serial python3-pygame
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build && source install/setup.bash
```

Both machines need the same `ROS_DOMAIN_ID` and must be on the same network.

## Run

Orange Pi (Pico plugged in over USB):

```bash
ros2 run rover_pkg onboard --ros-args -p port:=/dev/ttyACM0
```

Laptop:

```bash
ros2 run rover_pkg ground_station
```

Click the pygame window to give it focus, then press **Enter** to enable. The keys are listed at the top of `ground_station_node.py`.

## Safety

- The ground station publishes `ArmCommand` at 20 Hz. If it stops for 0.5 s, `onboard` sends `LINK_LOST` to the Pico.
- The Pico also stops on its own if heartbeats stop for 1 s, and it zeroes the motor if motor commands stop for 300 ms.
- If the Pico stops for any reason, the operator must press Enter again.
- The firmware ships with `DRY_RUN` on (no actuator signals). Servo calibration in `config.h` is still unfinished.
