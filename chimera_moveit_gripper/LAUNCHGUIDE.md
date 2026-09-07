# AGRIBOT Robot Setup & Launch Guide

This README provides the steps required to launch the **AGRIBOT software stack**, including MoveIt, ML-based inverse kinematics (IK), robot hardware, and the final IK solver.

## 1. Prerequisites

Before starting, ensure that:

* ROS is installed and configured.
* All required ROS packages and dependencies are installed.
* The AGRIBOT is powered ON.
* The robot hardware and required connections are properly connected.
* The ROS workspace is available and built successfully.

### Source the ROS Workspace

For each new terminal, source the workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

To avoid sourcing it manually in every terminal, add it to `~/.bashrc`:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Then reload the configuration:

```bash
source ~/.bashrc
```

> **Note:** Replace `~/catkin_ws` with the actual path to your ROS workspace if it is different.

---

# 2. Launch AGRIBOT

The complete system requires **four separate terminals**. Execute the following commands **in order**.

### Terminal 1 — AGRIBOT / MoveIt

```bash
roslaunch chimera_moveit_gripper demo.launch
```

This launches the AGRIBOT MoveIt environment and initializes the arm and gripper control components.

---

### Terminal 2 — ML-Based IK

```bash
roslaunch ml_onboard ml_ik.launch
```
This starts the onboard **machine-learning** node, along with **Transformation** node, which converts the detected tomato center into world coordinates used by the arm to plan ts motion.

---

### Terminal 3 — Robot Hardware

```bash
roslaunch chimera_moveit_gripper hardware.launch
```
This initializes the robot hardware interface (track wheel along with arm servo) and establishes communication with the physical AGRIBOT.

> **Important:** Ensure that the robot as well as MyCobot arm is powered ON and connected before launching the hardware interface.

---

### Terminal 4 — Run the IK Solver

```bash
rosrun chimera_moveit_gripper 2_ik_solver.py
```

This starts the final IK solver and provides the interface for commanding the robotic arm to move to the target location.

After the IK solver starts:

1. The solver will prompt for confirmation in the terminal.
2. Enter **`y`** to start planning the arm motion to the target location.
3. The IK solver checks whether the target pose is feasible and generates a motion plan.
4. If a valid plan is found, the arm executes the planned motion.
5. If the target is not feasible or the planning fails, the solver will ask whether you want to **replan**.
6. Enter **`y`** to attempt replanning or **`n`** to stop/skip the movement.

> **Important:** The `2_ik_solver.py` terminal is interactive. Keep this terminal visible during operation, as user input (`y`/`n`) is required to confirm motion planning and replanning.

---

## 3. System Interfaces

Once all four commands are running successfully, the AGRIBOT provides separate interfaces for **arm control, wheel control, and ML detection monitoring**.

### 🦾 Arm Control GUI

A dedicated GUI is available for controlling and monitoring the robotic arm.

The arm movement workflow is:

```text
Target Location
      ↓
IK Solver
      ↓
Enter `y` in IK Solver Terminal
      ↓
Check IK / Motion Feasibility
      ↓
 ┌───────────────┐
 │ Plan feasible?│
 └───────┬───────┘
         │
    ┌────┴────┐
   YES        NO
    │          │
    ▼          ▼
 Execute    Ask for
 movement   replanning
               │
             y / n
```

To move the arm to a target location:

* Enter **`y`** in the IK solver terminal when prompted.
* The system plans a trajectory to the target.
* If the target is feasible, the arm executes the planned trajectory.
* If planning fails, the system asks whether to replan.
* Enter **`y`** to replan or **`n`** to stop.

### 🚗 Wheel Control

A separate interface is provided for controlling the AGRIBOT mobile base.

Use the wheel-control interface to:

* Move the robot forward/backward.
* Control left/right movement.
* Position the AGRIBOT at the required location.

### 📷 Camera & ML Detection Visualization

A separate camera visualization window displays the live camera feed and ML detection results.

Use this interface to:

* Monitor the live camera feed.
* Verify detected objects.
* Observe ML detection results during robot operation.

> **In short:**
> **Arm GUI → Arm and target-position control**
> **IK Solver Terminal → Confirm/replan arm motion using `y`/`n`**
> **Wheel Control → Mobile-base movement**
> **Camera Visualization → Live camera + ML detection monitoring**

---

# 4. Complete Launch Sequence

Open **four terminals** and run:

| Terminal | Command                                            | Function                                |
| -------- | -------------------------------------------------- | --------------------------------------- |
| **1**    | `roslaunch chimera_moveit_gripper demo.launch`     | Launch MoveIt + arm/gripper environment |
| **2**    | `roslaunch ml_onboard ml_ik.launch`                | Start ML-based IK                       |
| **3**    | `roslaunch chimera_moveit_gripper hardware.launch` | Connect to robot hardware               |
| **4**    | `rosrun chimera_moveit_gripper 2_ik_solver.py`     | Start final IK solver                   |

### Execution Flow

```text
                    AGRIBOT SYSTEM
                          │
          ┌───────────────┼────────────────┐
          │               │                │
          ▼               ▼                ▼
     ARM CONTROL      WHEEL CONTROL    CAMERA VIEW
       GUI               GUI           + ML DETECTION
          │               │                │
          └───────────────┼────────────────┘
                          │
                          ▼
                    ROS SOFTWARE STACK
                          │
          ┌───────────────┼────────────────┐
          ▼               ▼                ▼
       MoveIt           ML-IK          Hardware
          │               │                │
          └───────────────┼────────────────┘
                          ▼
                    IK SOLVER
```

---

# 5. Verification

After launching all four terminals, verify that the required ROS nodes are running:

```bash
rosnode list
```

Verify that the required packages are available:

```bash
rospack find chimera_moveit_gripper
rospack find ml_onboard
```

The system is ready when:

* MoveIt/arm environment is running.
* ML-IK node is running.
* Hardware interface is connected.
* IK solver is running.
* Arm control GUI is available.
* Wheel control interface is available.
* Camera visualization shows the live feed and ML detections.

---

# 6. Troubleshooting

## ROS Package Not Found

If you get:

```text
[roslaunch] ERROR: package '...' not found
```

Source the workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

Then verify the package:

```bash
rospack find chimera_moveit_gripper
rospack find ml_onboard
```

If the package is still not found, ensure that the workspace has been built correctly.

---

## Robot Hardware Not Responding

Check:

* Robot power is ON.
* Required USB/network connections are active.
* No other process is controlling the robot.
* `hardware.launch` started without errors.

---

## IK Solver Not Starting

Check that the ML-IK and hardware nodes are running:

```bash
rosnode list
```

If required, restart the ML-IK node:

```bash
roslaunch ml_onboard ml_ik.launch
```

Then restart the IK solver:

```bash
rosrun chimera_moveit_gripper 2_ik_solver.py
```
---

# 7. Quick Start

If the ROS environment and robot are already configured, open **four terminals** and run the following commands in order:

```bash
# Terminal 1
roslaunch chimera_moveit_gripper demo.launch
```

```bash
# Terminal 2
roslaunch ml_onboard ml_ik.launch
```

```bash
# Terminal 3
roslaunch chimera_moveit_gripper hardware.launch
```

```bash
# Terminal 4
rosrun chimera_moveit_gripper 2_ik_solver.py
```

Once all four are running, use the **Arm GUI**, **Wheel Control**, and **Camera/ML Visualization** interfaces to operate and monitor the AGRIBOT.
