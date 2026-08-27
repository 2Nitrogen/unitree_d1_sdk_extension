# The examples deliberately consume the package the same way any other project
# would: no hardcoded include or link directories, just the imported target.
# If these build, the installed package is self-contained.

set(D1_EXAMPLES
  joint_angle_control
  multiple_joint_angle_control
  joint_enable_control
  arm_zero_control
  get_arm_joint_angle
  get_arm_command
)

foreach(ex ${D1_EXAMPLES})
  add_executable(${ex} ${ex}.cpp)
  target_link_libraries(${ex} PRIVATE unitree_arm)
endforeach()
