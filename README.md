# DAQ workspace

ROS 2 Humble workspace for acquiring Intel RealSense D455 and TI IWR6843ISK
sensor data.

The workspace contains separate D455 and IWR6843ISK bringup launch files and a
third, visualization-only launch that overlays both sensors in RViz. Rosbag
recording remains separate from visualization.

## Prerequisite

Install the ROS 2 RealSense wrapper before running the camera launch.

```bash
sudo apt update
sudo apt install \
  ros-humble-realsense2-camera \
  ros-humble-compressed-image-transport \
  ros-humble-compressed-depth-image-transport \
  ros-humble-depth-image-proc \
  ros-humble-rosbag2 \
  ros-humble-tf2-ros \
  python3-yaml
```

## Build

```bash
cd /home/hoyoung/workspace/daq_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start the D455

```bash
ros2 launch daq_bringup d455.launch.py
```

The default advanced-mode profile is `d455_MD_emitter_off.json`. Select the
high-accuracy profile without editing the launch file:

```bash
ros2 launch daq_bringup d455.launch.py \
  json_file_path:=/home/hoyoung/workspace/daq_ws/src/daq_bringup/config/d455/d455_HA.json
```

Use different ROS and RealSense Viewer configuration files when needed:

```bash
ros2 launch daq_bringup d455.launch.py \
  config_file:=/absolute/path/to/d455.yaml \
  json_file_path:=/absolute/path/to/d455.json
```

The default profile enables color, depth, infrared stereo, accelerometer,
gyroscope, interpolated IMU, and the D455 TF tree. It does not start a viewer,
depth alignment, or PointCloud2 generation. RGB runs at 640x480x15; depth and
infrared stereo run at 848x480x30 for the validated VINS and planned EGO input.

## Start the IWR6843ISK

Put the TI mmWave CLI profile at:

```text
/home/hoyoung/workspace/daq_ws/src/daq_bringup/config/iwr6843isk/radar.cfg
```

Build and source this workspace, then launch the radar:

```bash
source /opt/ros/humble/setup.bash
source /home/hoyoung/workspace/daq_ws/install/setup.bash
ros2 launch daq_bringup iwr6843isk.launch.py
```

The defaults are `/dev/ttyUSB0` for the 115200-baud CLI port and
`/dev/ttyUSB1` for the 921600-baud data port. Override them when enumeration
differs:

```bash
ros2 launch daq_bringup iwr6843isk.launch.py \
  cli_port:=/dev/ttyUSB2 data_port:=/dev/ttyUSB3
```

The driver publishes `sensor_msgs/msg/PointCloud2` on `/iwr6843_pcl` with
frame ID `iwr6843_frame`.

Start the radar with its standalone PointCloud2 configuration:

```bash
ros2 launch daq_bringup iwr6843isk.launch.py rviz:=true
```

## View the D455 and IWR6843ISK together

Start the two sensors in separate terminals first. Do not add `rviz:=true` to
the radar command when using the combined view.

```bash
# Terminal 1
ros2 launch daq_bringup d455.launch.py

# Terminal 2
ros2 launch daq_bringup iwr6843isk.launch.py

# Terminal 3
ros2 launch daq_bringup sensor_view.launch.py
```

The third launch does not start either sensor. While it is running, it:

- limits the 30 Hz raw depth input to at most 15 Hz;
- registers depth into the color camera frame;
- publishes the RGB-textured D455 cloud on
  `/camera/camera/depth/color/points`;
- publishes `camera_link -> iwr6843_frame` from the extrinsics YAML; and
- opens the saved combined RViz layout.

The camera launch keeps `pointcloud.enable: false`, so stopping the combined
view removes all visualization point-cloud processing. Change only the view
rate when a lower Jetson load is required:

```bash
ros2 launch daq_bringup sensor_view.launch.py pointcloud_rate:=10.0
```

The radar-camera transform in
`config/extrinsics/d455_iwr6843isk.yaml` has `calibrated: false`. It is an
initial mounting estimate for visualization, not a completed extrinsic
calibration.

## Record a DAQ bag

Start `cam` and `radar` first. Recording is a separate command and does not
start RViz or the visualization point-cloud pipeline.

```bash
ros2 launch daq_bringup bag_record.launch.py bag_name:=2m_0_test1
```

The output layout is:

```text
~/daq_bags/YYYYMMDD/<bag_name>_YYYYMMDD_HHMMSS/bag/
  bag_0.db3
  metadata.yaml
```

The recorder waits for one real message from every configured topic before it
starts. It refuses to record if the camera IMU, radar, or another required
stream is missing. Stop with `Ctrl-C` and wait until `Recording stopped` and
`process has finished cleanly` are printed.

The same terminal prints a two-second live rate summary while preflight and
recording are active:

```text
DAQ Hz: RGB=14.0/15Hz[OK] | Depth=28.5/30Hz[OK] | ... | Radar=30.0/30Hz[OK]
```

`LOW` means the measured rate is below 90% of the configured target;
`MISSING` means no message arrived during the run. Targets and monitored topics
are stored in `config/bag/rate_monitor.yaml`. The report interval can be
changed without editing the file:

```bash
ros2 launch daq_bringup bag_record.launch.py \
  bag_name:=test1 rate_report_period:=5.0
```

Recorded data is defined in `config/bag/record_topics.yaml`:

- compressed RGB and compressedDepth;
- RGB/depth/IR1/IR2 CameraInfo;
- raw IR1/IR2 stereo images;
- raw accelerometer and gyroscope messages;
- raw `/iwr6843_pcl`; and
- D455 `/tf_static`.

Raw RGB, raw depth, and the visualization-only RGB-D PointCloud2 are not
recorded. Rosbag storage compression is disabled because RGB and depth are
already image-compressed. The raw 848x480x30 IR stereo pair dominates storage;
a live camera-only test measured approximately 1.45 GiB/min.

Change the output root when required:

```bash
ros2 launch daq_bringup bag_record.launch.py \
  bag_name:=move_turn1 output_root:=/data/daq_bags
```
