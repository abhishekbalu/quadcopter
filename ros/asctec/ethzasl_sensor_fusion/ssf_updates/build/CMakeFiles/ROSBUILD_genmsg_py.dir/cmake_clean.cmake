FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ssf_updates/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ssf_updates/msg/__init__.py"
  "../src/ssf_updates/msg/_PositionWithCovarianceStamped.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
