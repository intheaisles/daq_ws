# IWR6843ISK configuration

Place the TI mmWave Out-of-Box Demo CLI configuration in this directory as:

```text
radar.cfg
```

The current publisher requires a legacy frame configuration containing at
least `profileCfg`, `frameCfg`, and `sensorStart`. Keep point-cloud output
enabled in `guiMonitor`.

To use another filename, pass its absolute path to the launch file:

```bash
ros2 launch daq_bringup iwr6843isk.launch.py \
  radar_config:=/absolute/path/to/profile.cfg
```
