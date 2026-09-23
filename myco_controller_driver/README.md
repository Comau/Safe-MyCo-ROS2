# MyCo controller driver

This package is the controller-backed replacement for the legacy SOEM hardware
plugin. Port 8893 is consumed by a dedicated state thread and port 8892 is used
for `StartServo` and `PushServoJ` commands. ROS joint units are radians;
the controller boundary defaults to degrees.

The SDK node exposes:

```text
/myco_sdk/set_enabled      std_srvs/srv/SetBool
/myco_sdk/electrify        std_srvs/srv/Trigger
/myco_sdk/initialize_controller std_srvs/srv/Trigger
/myco_sdk/blackout         std_srvs/srv/Trigger
/myco_sdk/set_ros_control  std_srvs/srv/SetBool
/myco_sdk/set_freedrive    std_srvs/srv/SetBool
/myco_sdk/set_force_freedrive std_srvs/srv/SetBool
/myco_sdk/stop             std_srvs/srv/Trigger
/myco_sdk/reset            std_srvs/srv/Trigger
/myco_sdk/pause            std_srvs/srv/Trigger
/myco_sdk/continue         std_srvs/srv/Trigger
/myco_sdk/set_override     myco_robot_msgs/srv/SetFloat64
/myco_sdk/set_tcp          myco_robot_msgs/srv/SetPose
/myco_sdk/configure_tcp    myco_robot_msgs/srv/ConfigureTcp
/myco_sdk/get_tcp_config   myco_robot_msgs/srv/GetTcpConfig
/myco_sdk/restore_default_tcp std_srvs/srv/Trigger
/myco_sdk/set_ucs          myco_robot_msgs/srv/SetPose
/myco_sdk/set_tcp_by_name  myco_robot_msgs/srv/SetString
/myco_sdk/set_ucs_by_name  myco_robot_msgs/srv/SetString
/myco_sdk/set_digital_io   myco_robot_msgs/srv/SetDigitalIO
/myco_sdk/set_analog_io    myco_robot_msgs/srv/SetAnalogIO
```

`set_ros_control(false)` stops the configured motion controller but leaves the
joint-state broadcaster and port 8893 state updates running. Entering freedrive
also stops the motion controller first. Exiting freedrive deliberately does not
restart an old trajectory; call `set_ros_control(true)` explicitly after the arm
is stable.

`set_freedrive` is the normal zero-force teaching mode and uses
`HRIF_GrpOpenFreeDriver`. `set_force_freedrive` is the distinct force-control
free-drive mode and uses `HRIF_SetForceFreeDriveMode`. The two modes are mutually
exclusive.

Published topics:

```text
/myco_sdk/realtime_state  myco_robot_msgs/msg/MycoRealtimeState (8893)
/myco_sdk/joint_states    sensor_msgs/msg/JointState             (8893)
/myco_sdk/robot_status    myco_robot_msgs/msg/MycoRobotStatus  (10004 JSON stream)
/myco_sdk/io_state        myco_robot_msgs/msg/MycoIOState       (10004 JSON stream)
/myco_sdk/end_io_state    myco_robot_msgs/msg/MycoEndIOState    (10004 JSON stream)
```

Joint values and angular TCP values in ROS messages are SI units (rad, rad/s).
TCP translations and linear velocities are converted from mm to m. `SetPose`
requests intentionally use the SDK-native units: mm and degrees.

Common SDK command examples:

```bash
ros2 service call /myco_sdk/set_tcp myco_robot_msgs/srv/SetPose \
  "{pose: [0.0, 0.0, 150.0, 0.0, 0.0, 0.0]}"

# Persist, select, and verify a named TCP in the controller configuration.
ros2 service call /myco_sdk/configure_tcp myco_robot_msgs/srv/ConfigureTcp \
  "{name: ROS_TCP, pose: [0.0, 0.0, 150.0, 0.0, 0.0, 0.0]}"

ros2 service call /myco_sdk/set_digital_io myco_robot_msgs/srv/SetDigitalIO \
  "{domain: box_do, index: 0, value: true}"

ros2 service call /myco_sdk/set_analog_io myco_robot_msgs/srv/SetAnalogIO \
  "{index: 0, mode: 0, value: 5.0}"

ros2 service call /myco_sdk/set_override myco_robot_msgs/srv/SetFloat64 \
  "{data: 0.5}"
```

Examples:

```bash
ros2 launch myco_robot_bringup myco_controller.launch.py \
  robot_model:=MyCo-5-0.80 robot_ip:=10.20.200.3 control_mode:=controller \
  update_rate:=250

ros2 service call /myco_sdk/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /myco_sdk/set_freedrive std_srvs/srv/SetBool '{data: true}'
ros2 service call /myco_sdk/set_freedrive std_srvs/srv/SetBool '{data: false}'
ros2 service call /myco_sdk/set_ros_control std_srvs/srv/SetBool '{data: true}'
```

`update_rate` follows the controller cycle: use `250` for a controller version
marked `4ms` and `1000` for a 1 ms controller. The direct
`myco_controller.launch.py` entry defaults to 250 Hz and accepts a one-shot
override. For normal use, prefer `myco_control.launch.py` and change only
`myco_control.update_rate` in
`myco_robot_bringup/config/myco_control.yaml`. The hardware reads the actual
8893 `cycle_time` during activation and aborts the complete launch when it does
not match the configured rate.

The position motion controller is initially loaded inactive. If the
first 10004 status frame reports enabled, unpaused, fault-free, and all joint
brakes released, the driver activates it automatically after loading completes.
If the robot is not ready at startup, enabling it later does not automatically
activate ROS control; call `set_ros_control(true)` explicitly. Loss of readiness
or of the 10004 stream deactivates it and requires another explicit activation
request after recovery.

