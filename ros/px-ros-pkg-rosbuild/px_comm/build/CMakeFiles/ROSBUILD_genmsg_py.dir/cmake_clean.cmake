FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/px_comm/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/px_comm/msg/__init__.py"
  "../src/px_comm/msg/_OpticalFlow.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
