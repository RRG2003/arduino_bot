```markdown
# 🤖 Arduinobot – A ROS2 Jazzy Based Educational Robot Platform

![Arduinobot](https://github.com/RRG2003/arduino_bot/blob/main/arduinobot.jpg?raw=true)

Arduinobot is a lightweight, modular, and fully ROS2-compatible robot platform designed for learning
robotics, motion planning, kinematics, and ROS2 fundamentals.  
This repository contains the complete stack — URDF, MoveIt2 configuration, Python/CPP nodes, custom
services, controllers, and utilities.

---

# 📂 Repository Structure

```

arduino_bot/
├── arduinobot_description/      # URDF + meshes
├── arduinobot_controller/       # ros2_control controllers + hardware interfacing
├── arduinobot_cpp_examples/     # C++ publisher/subscriber/service/action examples
├── arduinobot_msgs/             # Custom message + service definitions
├── arduinobot_utils/            # Utility Python nodes (angle conversions etc.)
├── arduinobot_moveit/           # MoveIt2 configuration package
└── install / build / log        # Colcon workspace build outputs

````

---

# 🦾 Features

✅ **ROS2 Jazzy native**  
✅ **URDF + SRDF robot model**  
✅ **MoveIt2 motion planning support (OMPL planners)**  
✅ **Custom ROS2 interfaces**  
✅ **C++ example nodes (pub/sub/services)**  
✅ **Python utility nodes**  
✅ **Controller support (ros2_control)**  
✅ **Fully modular and extendable**  

---

# 🚀 Getting Started

## ✅ 1. Clone the repository

```bash
git clone https://github.com/RRG2003/arduino_bot
cd arduino_bot
````

---

## ✅ 2. Install required ROS2 packages

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-moveit-planners-ompl
```

---

## ✅ 3. Build the workspace

```bash
colcon build
source install/setup.bash
```

---

# 🤖 Robot Description

The robot model is defined in:

```
arduinobot_description/urdf/arduinobot.urdf.xacro
```

It includes:

* Base
* Virtual arm with 4 joints
* Gripper
* STL meshes

The semantic description (SRDF) defines:

* Groups: `virtsual_arm`, `gripper`
* Predefined poses
* Collision settings

Located in:

```
arduinobot_moveit/config/arduinobot.srdf
```

---

# 📦 Custom ROS2 Interfaces

Package: `arduinobot_msgs`

### ✅ Services

* `EulerToQuaternion.srv`
* `QuaternionToEuler.srv`

Used by utility nodes for angle conversions.

---

# 🧠 Python Utilities — `arduinobot_utils`

Includes:

* `angle_conversions.py`
  Converts Euler ⇆ Quaternion using custom services.

Run with:

```bash
ros2 run arduinobot_utils angle_conversions
```

---

# 🧪 C++ Examples — `arduinobot_cpp_examples`

Includes:

✅ Publishers
✅ Subscribers
✅ Service clients/servers
✅ Action clients

Great for learning ROS2 fundamentals in C++.

---

# 🦾 MoveIt2 Motion Planning – `arduinobot_moveit`

This package provides:

* MoveIt SRDF
* Kinematics config
* Joint limits
* Controllers
* OMPL planner configs
* RViz2 MotionPlanning setup

Launch with:

```bash
ros2 launch arduinobot_moveit demo.launch.py
```

This opens RViz with:

✅ Robot model
✅ MotionPlanning plugin
✅ OMPL planners (RRTConnect, PRM, etc.)

---

# 🔧 ros2_control Configuration

The controllers are defined in:

```
arduinobot_controller/config/*.yaml
```

Launch your controller with:

```bash
ros2 launch arduinobot_controller controller.launch.py
```

---

# 🧪 Testing Services

Example:

```bash
ros2 service call /euler_to_quaternion arduinobot_msgs/srv/Eulertoquaternion "{roll: 0.0, pitch: 0.0, yaw: 1.57}"
```

---

# 🗂️ Launch Files

### ✅ MoveIt:

```
ros2 launch arduinobot_moveit demo.launch.py
```

### ✅ Controllers:

```
ros2 launch arduinobot_controller controller.launch.py
```

### ✅ Utilities:

```
ros2 run arduinobot_utils angle_conversions
```

---

# 🛠️ Future Improvements

* Add forward/inverse kinematics implementation
* Add real hardware communication layer
* Add Gazebo / Isaac Sim simulation
* Add MoveIt Task Constructor

---

# 🙌 Contributing

PRs and issues are welcome.
If you want more examples or want to extend the robot, feel free to open a pull request!

---

# 📜 License

MIT License.
Use freely for learning and development.

---

# ⭐ Support the Project

If you find this useful, please ⭐ star the repository!

```

---


Just tell me!
```
