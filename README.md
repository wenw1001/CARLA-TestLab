# CARLA-TestLab

This repository is used for testing customized maps, vehicle control scripts, dataset collection pipelines, and vehicle trajectory recording functions within the CARLA simulation environment.  
It serves as a sandbox for validating and iterating different functionalities related to autonomous driving development.

## Features
- Custom map loading and testing
- Vehicle control via Python API
- Dataset collection (images, vehicle states, etc.)
- Trajectory recording and analysis
- Experimental scripts based on CARLA official examples

## Requirements
- Ubuntu 20.04
- Unreal Engine 4
- CARLA Simulator (version 0.9.15)
- Python 3.7 ~ 3.10
- Python packages:
  ```bash
  pip install -r requirements.txt
  ```
- (Optional) NVIDIA GPU for faster simulation rendering

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/CARLA-TestLab.git
   cd CARLA-TestLab
   ```

2. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

3. Make sure your CARLA simulator is running:
   ```bash
   # make launch
   ./CarlaUE4.sh  # Linux
   CarlaUE4.exe   # Windows
   ```

## Usage

Run the example scripts:

- **Manual vehicle control**:
  ```bash
  python scripts/manual_control.py
  ```

- **Dataset collection**:
  ```bash
  python scripts/data_collector.py
  ```

- **Trajectory recording**:
  ```bash
  python scripts/trajectory_recorder.py
  ```

_(Paths and script names can be adjusted according to your project structure.)_

## Project Structure

```bash
CARLA-TestLab/
├── Unreal/CarlaUE4/Content/CustomMaps/JSRoad3/                   # Custom maps (if any)
├── PythonAPI/examples/                 # Python scripts for control, data collection, etc.
│   ├── utilsWen/general.py
│   ├── manual_control_wen.py
│   ├── wen_autocontrol.py
│   ├── wen_generate_vehicles.py
│   ├── wen_get_vehicle_info.py
│   └── wen_png_to_mov.py
├── requirements.txt
└── README.md
```

## TODO

- [ ] Add more advanced vehicle control strategies (e.g., PID, MPC)
- [ ] Improve dataset saving formats
- [ ] Visualize vehicle trajectories
- [ ] Upload example custom maps
