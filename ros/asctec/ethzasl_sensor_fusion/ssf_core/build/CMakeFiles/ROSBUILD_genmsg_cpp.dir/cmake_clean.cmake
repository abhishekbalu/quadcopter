FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ssf_core/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ssf_core/DoubleArrayStamped.h"
  "../msg_gen/cpp/include/ssf_core/ext_ekf.h"
  "../msg_gen/cpp/include/ssf_core/ext_imu.h"
  "../msg_gen/cpp/include/ssf_core/ext_state.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
