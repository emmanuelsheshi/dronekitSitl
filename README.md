# PFGCS – Python Flight Ground Control Scripts

A collection of DroneKit-Python scripts for controlling ArduPilot vehicles in
simulation (SITL) or with real hardware.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Step 1: Create Python 3.7 Environment](#step-1-create-python-37-environment)
3. [Step 2: Install MAVProxy (Outside Conda)](#step-2-install-mavproxy-outside-conda)
4. [Step 3: The 3-Terminal Workflow](#step-3-the-3-terminal-workflow)
5. [Project Scripts](#project-scripts)
6. [Connecting QGroundControl / Mission Planner](#connecting-qgroundcontrol--mission-planner)
7. [Troubleshooting](#troubleshooting)
8. [Files Reference](#files-reference)

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| **Windows 10/11** | Tested on Windows |
| **Conda** (Anaconda / Miniconda) | For Python environment management |
| **MAVProxy** | Installed separately via the official website |

---

## Step 1: Create Python 3.7 Environment

DroneKit requires **Python 3.7**. Create a dedicated Conda environment and
install the required packages **inside** it.

```powershell
# 1. Create the environment with Python 3.7
conda create -n dronkitGcs2 python=3.7 -y

# 2. Activate the environment
conda activate dronkitGcs2

# 3. Install DroneKit and DroneKit-SITL
pip install dronekit dronekit-sitl

# 4. (Optional) Install pymavlink and pyttsx3 for TTS
pip install pymavlink pyttsx3
```

You can also recreate the environment from the included YAML file:

```powershell
conda env create -f dronkitGcs2.yaml
conda activate dronkitGcs2
```

---

## Step 2: Install MAVProxy (Outside Conda)

MAVProxy is installed **outside** the Conda environment using its official
Windows installer.

### Download & Install

1. Go to the official MAVProxy website:
   **https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html**

2. Download the **Windows installer** (`.exe`).

3. Run the installer and follow the prompts.

4. After installation, MAVProxy is available system-wide. You can run it from
   **any terminal** (PowerShell, CMD) without activating Conda.

### Verify Installation

Open a **new** PowerShell window (not inside Conda) and run:

```powershell
mavproxy --version
```

You should see the MAVProxy version number.

---

## Step 3: The 3-Terminal Workflow

You need **3 terminals** running simultaneously:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ARCHITECTURE                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Terminal 1              Terminal 2                Terminal 3              │
│   ───────────             ───────────               ───────────             │
│   SITL Drone              MAVProxy                  Python Script           │
│   (dronekit-sitl)         (message hub)             (your code)             │
│        │                       │                         │                  │
│        │   tcp:5760            │                         │                  │
│        └──────────────────────►│                         │                  │
│                                │   udp:14551             │                  │
│                                ├────────────────────────►│                  │
│                                │                         │                  │
│                                │   udp:14550             │                  │
│                                └────────────────────────►│ QGC / Mission    │
│                                                          │ Planner          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

### Terminal 1 – SITL (Simulated Drone)

This terminal runs the simulated ArduCopter on **port 5760**.

```powershell
# Activate the Conda environment
conda activate dronkitGcs2

# Start SITL with a custom home location
dronekit-sitl copter --home=6.9732271,3.6842822,10,0
```

| Argument | Meaning |
|----------|---------|
| `6.9732271` | Home latitude |
| `3.6842822` | Home longitude |
| `10` | Home altitude (metres AMSL) |
| `0` | Heading (degrees) |

SITL listens on **tcp:127.0.0.1:5760**.

> Keep this terminal open. The drone is running here.

---

### Terminal 2 – MAVProxy (Message Hub)

This terminal runs MAVProxy, which:
- **Receives** from SITL on port 5760
- **Shares** to QGroundControl/Mission Planner on UDP 14550
- **Shares** to your Python script on UDP 14551

```powershell
# No need to activate Conda – MAVProxy is installed system-wide
mavproxy --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

| Flag | Purpose |
|------|---------|
| `--master=tcp:127.0.0.1:5760` | Connect to SITL |
| `--out=udp:127.0.0.1:14550` | Forward to QGC / Mission Planner |
| `--out=udp:127.0.0.1:14551` | Forward to your Python script |

> Keep this terminal open. MAVProxy is the communication bridge.

---

### Terminal 3 – Python Script (Your Code)

This terminal runs your Python code that **controls SITL via MAVProxy**.

```powershell
# Activate the Conda environment
conda activate dronkitGcs2

# Run a script (connects to MAVProxy on UDP 14551)
python dropTest.py --connect udp:127.0.0.1:14551
```

Or for `simpleDrone.py`:

```powershell
python simpleDrone.py --connect udp:127.0.0.1:14551
```

Or for `dronetakeoff.py`:

```powershell
python dronetakeoff.py --connect udp:127.0.0.1:14551
```

---

## Project Scripts

| Script | Description |
|--------|-------------|
| `dronetakeoff.py` | Simple arm, takeoff, hover, and land demo. |
| `simpleDrone.py` | Flies a square pattern of waypoints, then RTL and disarm. |
| `dropTest.py` | Flies to drop waypoint (6.9778007, 3.6628544), announces **"droping package"** via TTS, then RTL and disarm. |

---

## Connecting QGroundControl / Mission Planner

With MAVProxy running, your ground station software can connect to the drone.

### QGroundControl

1. Open QGroundControl
2. Go to **Application Settings → Comm Links**
3. Add a new link:
   - Type: **UDP**
   - Port: **14550**
4. Connect – you should see the drone on the map

### Mission Planner

1. Open Mission Planner
2. Select **UDP** from the connection dropdown
3. Click **Connect**
4. Enter port **14550** when prompted

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **"ModuleNotFoundError: No module named 'dronekit'"** | Run `conda activate dronkitGcs2` first. |
| **"Link not running"** | Ensure SITL (Terminal 1) and MAVProxy (Terminal 2) are running. |
| **Script hangs at "Waiting for vehicle to initialise..."** | GPS lock can take 10–30 s in SITL. Wait for `EKF2 IMU0 is using GPS` in MAVProxy. |
| **No audio from `dropTest.py`** | Install `pyttsx3` (`pip install pyttsx3`). |
| **Port already in use** | Close orphan SITL/MAVProxy processes or restart terminals. |
| **MAVProxy not found** | Ensure MAVProxy installer completed. Open a new terminal and try `mavproxy --version`. |

---

## Files Reference

```
pfgcs/
├── dronkitGcs2.yaml   # Conda environment export
├── dronetakeoff.py    # Basic takeoff demo
├── dropTest.py        # Drop-waypoint + TTS demo
├── simpleDrone.py     # Multi-waypoint square pattern
├── mav.parm           # MAVLink parameter file (auto-generated)
├── mav.tlog           # Telemetry log (auto-generated)
└── readme.md          # ← You are here
```

---

## Quick Reference Commands

```powershell
# === Terminal 1: SITL ===
conda activate dronkitGcs2
dronekit-sitl copter --home=6.9732271,3.6842822,10,0

# === Terminal 2: MAVProxy ===
mavproxy --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

# === Terminal 3: Python Script ===
conda activate dronkitGcs2
python dropTest.py --connect udp:127.0.0.1:14551
```

---

## License

This project is provided for educational purposes. DroneKit is licensed under
Apache 2.0. See upstream repositories for license details.

