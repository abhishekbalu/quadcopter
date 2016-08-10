FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/laser/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/laser/full_pose.h"
  "../msg_gen/cpp/include/laser/Attitude.h"
  "../msg_gen/cpp/include/laser/EstimateSingle.h"
  "../msg_gen/cpp/include/laser/EstimateMulti.h"
  "../msg_gen/cpp/include/laser/state.h"
  "../msg_gen/cpp/include/laser/Estimate.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
