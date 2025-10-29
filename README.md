# Quadrapet V3 Codebase

# Deploying to real robot
Follow instructions here to flash your Raspberry Pi 5 with our custom image.

# Deploying to simulated robot on development machine (x86 Ubuntu 24)
## Install 
```sh
sudo apt install git-lfs
git lfs install
git clone --recurse-submodules
./install_dev_dependencies.sh
```

## Build
```sh
cd ros2_ws
source build.sh
```

# Docs

Please see the docs!

# Notes
* Camera FPS is 10hz by default. Adjustable in `ros2_ws/src/neural_controller/launch/config.yaml` with the `FrameDurationLimits: [100000, 100000]` parameter

# Development

## Adding animations

1. Hold L1 until BAG status icon turns green to indicate mcap bag recording in process
1. Move Quadrapet through desired motion
1. Press R1 to stop recording
1. View recorded mcap file in foxglove to verify animation
1. Use `scripts/mcap_to_csv.py [path_to_mcap] -s ABSOLUTE_START_TIME -e ABSOLUTE_END_TIME` to convert to csv
1. Copy csv to ros2_ws/src/animation_controller_py/launch/animations
1. Update quadrapetster.py with animation nickname
1. If editing animation frame rate or fade time, make sure to edit both config.yaml and quadrapetster.yaml

## Camera
Launch mock camera and detection nodes so you can experiment with vision with simulated robot
```sh
ros2 launch hailo detection_with_mock_camera_launch.py
```

Launch Foxglove bridge so you can see detections in Foxglove studio
```sh
ros2 run foxglove_bridge foxglove_bridge
```
