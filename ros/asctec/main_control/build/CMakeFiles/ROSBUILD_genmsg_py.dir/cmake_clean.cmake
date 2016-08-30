file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/main_control/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/main_control/msg/__init__.py"
  "../src/main_control/msg/_string_stamped.py"
  "../src/main_control/msg/_state_command.py"
  "../src/main_control/msg/_imu_acc.py"
  "../src/main_control/msg/_estimate.py"
  "../src/main_control/msg/_imu.py"
  "../src/main_control/msg/_full_state.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
