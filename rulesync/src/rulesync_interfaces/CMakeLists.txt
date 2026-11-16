cmake_minimum_required(VERSION 3.8)
project(rulesync_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RuleSpec.msg"
  "msg/StagedSet.msg"
  "msg/BridgeVoteMsg.msg"
  "msg/RuleVolume.msg"
  "msg/ExecutionAuthorization.msg"
  "msg/RuleGuardState.msg"
  "msg/VolumeAuthority.msg"
  "srv/ProposeRule.srv"
  "srv/CommitRule.srv"
  "srv/RemoveRule.srv"
  "srv/ListRules.srv"
  "srv/BridgeVote.srv"
  "srv/BridgeState.srv"
  "srv/ProposeVolume.srv"
  "srv/CommitVolume.srv"
  "srv/RemoveVolume.srv"
  "srv/ListVolumes.srv"
  "srv/BridgeVoteVolume.srv"
  "srv/BridgeVolumeState.srv"
  "srv/GetVolumeAuthority.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_package()
