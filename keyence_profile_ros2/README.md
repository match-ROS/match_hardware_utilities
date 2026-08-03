# Keyence profile ROS 2

This package establishes the ROS 2-compatible output contract of the former
LJ-X8400 print driver without claiming that a replay source is a live sensor
driver.  It publishes the legacy-compatible topics, all in SI units:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/profiles` | `sensor_msgs/msg/PointCloud2` | A one-row `(x, 0, z)` laser profile in the sensor frame. |
| `/profiles_float` | `std_msgs/msg/Float32MultiArray` | Raw profile heights (`NaN` represents invalid returns). |
| `/profiles_pitch_m` | `std_msgs/msg/Float32` | Lateral pitch in metres. |

Run a deterministic offline source with:

```bash
ros2 launch keyence_profile_ros2 keyence_profile_replay.launch.py \
  fixture_file:=/path/to/nominal_profiles.jsonl loop:=true
```

The later live LJ-X8400 adapter belongs here as well, behind this same topic
contract. `ljx8_profile_driver` now implements the vendor-SDK polling path;
the proprietary `libljxacom.so` must be supplied as `library_path` at runtime:

```bash
ros2 run keyence_profile_ros2 ljx8_profile_driver --ros-args \
  -p library_path:=/opt/keyence/libljxacom.so -p host:=192.168.12.88
```

It reconnects after SDK/open/read failures (with a configurable retry delay)
and publishes diagnostics at `/ljx8_profile_driver/diagnostics`. A connected
controller is not considered healthy unless a new profile has arrived within
`max_input_age`; `profile_age_sec` is published with the diagnostic. The
library is not redistributed by this package. A fake SDK test exercises
IPv4/port validation, open, optional measurement start, raw-buffer profile
decoding, stop/close cleanup, and the stale-profile watchdog with no controller
or shared library. Physical-controller compatibility, profile timing and
calibration remain required before it is permitted in a printing profile.
Map-frame TF and contour/error computation remain consumers in
`match_additive_manufacturing_ros2`.
