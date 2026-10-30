cmake_minimum_required(VERSION 3.8)
project(ebva_control_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RcInput.msg" "msg/ActuatorCommand.msg" "msg/SelectedControl.msg" "msg/DriveEffort.msg" "msg/FtgPdCommand.msg"
  "msg/LongitudinalEffortSteering.msg" "msg/AccelerationYawRateStamped.msg" "msg/CbfYawRateCommand.msg"
  "msg/FrenetLateralErrorStamped.msg"
  "msg/AccelerationSteeringAngleStamped.msg"
  "msg/CbfYawRateMarginCommand.msg"
  "msg/CbfSteeringTangentCommand.msg"
  "msg/TrackClassificationStamped.msg"
  DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
