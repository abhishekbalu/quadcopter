FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cam/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cam/msg/__init__.py"
  "../src/cam/msg/_imu_acc.py"
  "../src/cam/msg/_QuadPose.py"
  "../src/cam/msg/_state_command.py"
  "../src/cam/msg/_full_state.py"
  "../src/cam/msg/_imu.py"
  "../src/cam/msg/_detections.py"
  "../src/cam/msg/_QuadPoseList.py"
  "../src/cam/msg/_string_stamped.py"
  "../src/cam/msg/_estimate.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
