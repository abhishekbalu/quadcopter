file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/laser/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/laser/msg/__init__.py"
  "../src/laser/msg/_state.py"
  "../src/laser/msg/_EstimateMulti.py"
  "../src/laser/msg/_Estimate.py"
  "../src/laser/msg/_full_pose.py"
  "../src/laser/msg/_EstimateSingle.py"
  "../src/laser/msg/_Attitude.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
