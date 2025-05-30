cmake_minimum_required(VERSION 3.5)
project(leap_msgs)

# 必要なパッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# カスタムメッセージ型の設定
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hand.msg"
  "msg/Hands.msg"
  "msg/Finger.msg"
  "msg/Arm.msg"
  "msg/Bone.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# エクスポート設定
ament_export_dependencies(rosidl_default_runtime)
ament_package()