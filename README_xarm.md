# UFACTORY xArm 7 Simulation & Real Robot Control

A fully self-contained ROS 2 Humble workspace for controlling the UFACTORY xArm 7 robotic arm in both Gazebo simulation and real-world hardware. This repository solves common environment conflicts, Gazebo loading issues, and provides easy-to-use launch scripts.

## 🚀 Features

*   **Dual Mode:** Dedicated scripts for both Gazebo Simulation (`launch_simulation.sh`) and Real Hardware Control (`launch_real_robot.sh`).
*   **ROS 2 Isolation:** Automatically configures `ROS_DOMAIN_ID=42` to prevent interference from other ROS 2 projects (e.g., Humanoid/Panda robots) running on the same network.
*   **Gazebo Fixes:** Explicitly sets `GAZEBO_MODEL_PATH` to prevent Gazebo from hanging on online database downloads.
*   **Conda Compatibility:** Automatically strips Anaconda/Miniforge libraries from the path during execution to prevent `libssl`/`libcurl` conflicts with Gazebo's `spawn_entity`.
*   **MoveIt Integration:** Pre-configured with RViz markers and MoveIt 2 Motion Planning capabilities.

---

## 🛠️ Installation & Setup

### Prerequisites
*   Ubuntu 22.04
*   ROS 2 Humble
*   Git

### Clone & Build
Clone the repository and run the setup script to install all dependencies and build the workspace:

```bash
git clone https://github.com/Charan-mdnl/UFACTORY-xArm-7-WITH-MOVE-IT.git ~/xarm7_moveit_repo
cd ~/xarm7_moveit_repo
chmod +x setup.sh
./setup.sh
```

---

## 💻 Running the Gazebo Simulation

If you want to test trajectories without the physical robot:

```bash
cd ~/xarm7_moveit_repo
./launch_simulation.sh
```

**What this script does:**
1. Isolates the ROS 2 network.
2. Strips Conda environments from the library path.
3. Points Gazebo to local offline models.
4. Launches Gazebo with the xArm 7 beside a table.
5. Launches RViz with the MoveIt interactive markers enabled.

---

## 🤖 Running the Physical Robot

### 1. Hardware Connection
1. Connect the **50V 50A Battery** to the `V+` and `GND` pins on the green terminal block of the DC1300 Controller Box.
2. Connect the **Ethernet Cable** from the DC1300 LAN port directly to your laptop.
3. **Important:** Release the physical E-Stop button on the robot's base.

### 2. Network Configuration
The xArm controller has a default static IP of `192.168.1.230`. You must put your laptop on the same subnet:
```bash
# Example command (replace enp1s0 with your ethernet interface)
sudo ip addr add 192.168.1.100/24 dev enp1s0
sudo ip link set enp1s0 up
```
Verify the connection by running: `ping 192.168.1.230`

### 3. Enable the Robot
1. Open your browser and navigate to UFACTORY Studio: `http://192.168.1.230:18333`
2. Ensure there are no error codes.
3. Click **Enable Robot**.

### 4. Launch MoveIt Control
Once the robot is enabled, run the real-robot script:
```bash
cd ~/xarm7_moveit_repo
./launch_real_robot.sh
```

### 5. Moving the Robot
1. When RViz opens, look at the **MotionPlanning** panel on the bottom left.
2. Under the **Planning** tab, set **Velocity Scaling** and **Acceleration Scaling** to **`0.1`** (10% speed) for safety.
3. Check the `Query Goal State` and `Query Start State` boxes.
4. Drag the interactive marker ball at the end of the robot arm to your desired position.
5. Click **Plan** to preview the trajectory.
6. Keep your hand near the E-Stop button and click **Execute**.

---

## 🐛 Troubleshooting

*   **MoveIt crashes on startup with "Joint not found" errors:** 
    You have another ROS 2 project running in the background. The `launch_*.sh` scripts use `ROS_DOMAIN_ID=42` to isolate the network. Do not manually launch the python files without setting the domain ID.
*   **Gazebo gets stuck on "Connecting to model database":**
    Gazebo is trying to download models from the internet. Run `./launch_simulation.sh` as it forces Gazebo to use the local offline model path.
*   **RViz interactive markers are missing:**
    The default UFACTORY config hides them. We have modified `planner.rviz` to set `Interactive Marker Size: 0.2` and enable the query states by default.
*   **Script gives "Permission Denied":**
    Run `chmod +x launch_simulation.sh launch_real_robot.sh`
