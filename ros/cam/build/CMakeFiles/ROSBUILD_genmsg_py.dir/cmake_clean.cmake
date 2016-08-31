FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cam/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cam/msg/__init__.py"
  "../src/cam/msg/_QuadPose.py"
  "../src/cam/msg/_detections.py"
  "../src/cam/msg/_QuadPoseList.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
