FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/sensor_fusion_comm/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/sensor_fusion_comm/DoubleArrayStamped.h"
  "../msg_gen/cpp/include/sensor_fusion_comm/ExtState.h"
  "../msg_gen/cpp/include/sensor_fusion_comm/ExtEkf.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
