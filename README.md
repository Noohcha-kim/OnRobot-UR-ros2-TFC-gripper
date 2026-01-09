# OnRobot Gripper ROS2 Driver

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

Universal Robots Tool I/O based ROS2 driver for OnRobot grippers with Modbus RTU communication.

**Supported Grippers:**
- **2FG7 / 2FG14** - 2-Finger Electric Gripper
- **RG2 / RG6** - 2-Finger Adaptive Gripper
- **VG10 / VGC10** - Vacuum Gripper (2-channel)
- **3FG15 / 3FG25** - 3-Finger Adaptive Gripper
- **MG10** - Magnetic Gripper

---

## ⚠️ DISCLAIMER

**This driver is an unofficial open-source project.**

- **NOT officially supported by OnRobot**
- **ALL responsibility for using this software lies with the user**
- The developers and OnRobot are **NOT liable for gripper malfunction, damage, product failure, or any other issues**
- **Thorough testing is recommended before production use**
- **Follow safety regulations** and ensure **emergency stop systems** are in place

**THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.**

Please read the [LICENSE](LICENSE) file before use.

---

## 📋 Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [UR Robot Integration](#ur-robot-integration)
- [API Documentation](#api-documentation)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## ✨ Features

- ✅ **Automatic Gripper Detection** - Product code-based identification
- ✅ **Range Validation** - Automatic safety range checking per gripper
- ✅ **Error Recovery** - Automatic retry and reconnection
- ✅ **Real-time State** - 50Hz state updates
- ✅ **UR Robot Integration** - Tool I/O communication
- ✅ **ROS2 Native** - Action/Service interfaces

---

## 📦 Requirements

### Required
- **ROS2 Humble** (Ubuntu 22.04)
- **Universal Robots UR Series** (UR3e, UR5e, UR10e, UR16e, etc.)
- **OnRobot Gripper** (see supported list above)
- **socat** - Virtual serial port creation
- **External Control.urcap** 
- **RS485.urcap**
- 
```bash
sudo apt install socat
```

### Optional (For Robot Integration)
- **ur_robot_driver** - UR Robot ROS2 driver

```bash
sudo apt install ros-humble-ur-robot-driver
```

---

## 🚀 Installation

### 1. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/Noohcha-kim/OnRobot-UR-ros2-TFC-gripper.git
```

### 3. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ⚙️ Configuration

### 1. UR Robot Tool Communication Setup

**On UR Teach Pendant:**

1. **Installation → General**
   - Tool I/O Interface → Cotrolled by User → Communication Interface
   
2. **Installation → Communication**
   - RS485 Baudrate → **1000000** (or 115200)
   - RS485 Parity → **Even**
   - RS485 Stop Bits → 1


### 2. Check IP Address

Check your UR Robot IP address (e.g., `192.168.1.100`)

### 3. Start socat (Tool I/O Tunneling)

**Using default IP (192.168.56.101):**
```bash
cd ~/ros2_ws/src/onrobot_gripper_driver/scripts
./start_socat.sh
```

**Using custom IP:**
```bash
./start_socat.sh 192.168.1.100
```

**Using custom IP + port:**
```bash
./start_socat.sh 192.168.1.100 54321
```

---

## 🎮 Usage

### Architecture

**Important: socat and gripper driver run completely independently.**

```
┌─────────────────┐
│  socat          │ ← Start first (runs in background)
│  (Tool I/O)     │
└─────────────────┘
        ↓
┌─────────────────┐
│ Gripper Driver  │ ← Start second (separate terminal)
│ (ROS2 Node)     │
└─────────────────┘
```

### Step-by-Step Launch

**Step 1: Start socat (runs in background)**
```bash
cd ~/ros2_ws/src/onrobot_gripper_driver/scripts
./start_socat.sh YOUR_ROBOT_IP
```

**Step 2: Launch gripper driver (new terminal)**
```bash
ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py
```

**Step 3: Test commands**
```bash
# Grip
ros2 service call /onrobot_gripper_node/grip onrobot_gripper_driver/srv/Grip \
  "{target_position: 50.0, force: 40.0, speed: 50.0}"

# Release
ros2 service call /onrobot_gripper_node/release onrobot_gripper_driver/srv/Release "{}"
```

**To Stop:**
```bash
# Stop gripper driver: Ctrl+C in terminal
# Stop socat: ./stop_socat.sh
```

### Command Examples

#### Grip (2FG7 / RG2 example)

```bash
ros2 service call /onrobot_gripper_node/grip onrobot_gripper_driver/srv/Grip \
  "{target_position: 50.0, force: 40.0, speed: 50.0}"
```

#### Release

```bash
ros2 service call /onrobot_gripper_node/release onrobot_gripper_driver/srv/Release "{}"
```

#### Monitor State

```bash
ros2 topic echo /onrobot_gripper_node/state
```

---

## 🤖 UR Robot Integration

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from onrobot_gripper_driver.srv import Grip, Release

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
        # UR Robot
        self.robot = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Gripper
        self.grip_client = self.create_client(Grip, '/onrobot_gripper_node/grip')
        self.release_client = self.create_client(Release, '/onrobot_gripper_node/release')
    
    def pick_and_place(self):
        # 1. Move to pick position
        self.move_robot([0.5, -1.2, 1.3, -1.67, -1.57, 0.0])
        
        # 2. Grip
        req = Grip.Request()
        req.target_position = 40.0
        req.force = 60.0
        self.grip_client.call_async(req)
        
        # 3. Move to place position
        self.move_robot([-0.5, -1.2, 1.3, -1.67, -1.57, 0.0])
        
        # 4. Release
        self.release_client.call_async(Release.Request())

def main():
    rclpy.init()
    node = PickAndPlace()
    node.pick_and_place()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

More examples: See [examples/](examples/) folder

---

## 📚 API Documentation

### Services

#### `/onrobot_gripper_node/grip`

**2FG / RG Grippers:**
```yaml
target_position: float  # Target width (mm)
force: float           # Grip force (N)
speed: float           # Speed (%)
wait_for_completion: bool  # Wait until complete
grip_mode: int        # 0=External, 1=Internal (2FG only)
```

**VG Grippers:**
```yaml
target_position: float  # Vacuum strength (%)
channel_mask: int[]    # Channel selection [0, 1]
wait_for_completion: bool
```

**3FG Grippers:**
```yaml
target_position: float  # Diameter (mm)
force: float           # Force (N)
grip_mode: int        # 0=External, 1=Internal
```

#### `/onrobot_gripper_node/release`

```yaml
wait_for_completion: bool
channel_mask: int[]  # VG only
```

### Topics

#### `/onrobot_gripper_node/state` (50Hz)

```yaml
gripper_model: string    # "2FG7", "RG2", etc.
position_mm: float       # Current position (mm)
force_n: float          # Current force (N)
status: uint8           # Status bits
is_busy: bool           # In motion
is_gripping: bool       # Grip detected
channel_vacuum_pct: float[]  # VG only
```

---

## 🔧 Gripper Specifications

### 2FG7
- **Width:** 0 ~ 110 mm
- **Force:** 20 ~ 120 N
- **Speed:** 10 ~ 100 %

### 2FG14
- **Width:** 0 ~ 140 mm
- **Force:** 40 ~ 250 N
- **Speed:** 10 ~ 100 %

### RG2
- **Width:** 0 ~ 110 mm
- **Force:** 3 ~ 40 N
- **Speed:** 10 ~ 100 %

### RG6
- **Width:** 0 ~ 160 mm
- **Force:** 20 ~ 120 N
- **Speed:** 10 ~ 100 %

### VG10 / VGC10
- **Vacuum:** 0 ~ 100 %
- **Channels:** 2 (A, B)

### 3FG15
- **Diameter:** 0 ~ 150 mm
- **Force:** 50 ~ 500 N

### 3FG25
- **Diameter:** 0 ~ 250 mm
- **Force:** 50 ~ 500 N

### MG10
- **Strength:** 0 ~ 100 %

---

## 🐛 Troubleshooting

### socat fails to start

```bash
# Check IP connection
ping YOUR_ROBOT_IP

# Check port
telnet YOUR_ROBOT_IP 54321
```

### Gripper not detected

```bash
# Check product code
ros2 topic echo /onrobot_gripper_node/state --once

# Check logs
ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py
```

### Communication errors

```bash
# Restart socat
cd ~/ros2_ws/src/onrobot_gripper_driver/scripts
./stop_socat.sh
./start_socat.sh YOUR_ROBOT_IP

# Restart driver
ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py
```

---

## 🤝 Contributing

Bug reports and contributions are welcome!

### Bug Reports

Please report issues on **GitHub Issues** with the following information:

1. **Environment:**
   - ROS2 version
   - UR Robot model
   - Gripper model
   - OS (Ubuntu version)

2. **Problem Description:**
   - Steps to reproduce
   - Expected behavior
   - Actual behavior
   - Error logs

3. **Additional Information:**
   - Launch file configuration
   - Parameter settings
   - Related code

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

---

## 📖 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

## 📄 License

Apache License 2.0 - See [LICENSE](LICENSE) file

---

## 👤 Author

CHAHUN

---

## 🙏 Acknowledgments

- OnRobot - Gripper hardware and documentation
- Universal Robots - Tool I/O interface
- ROS2 Community

---

## 📞 Contact

- **Email:** minseung2201@gmail.com

---

**⚠️ REMINDER: ALL responsibility for using this software lies with the user. Follow safety regulations and conduct thorough testing before use.**
