FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/height_quad/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/height_quad/full_pose.h"
  "../msg_gen/cpp/include/height_quad/EstimateSingle.h"
  "../msg_gen/cpp/include/height_quad/Estimate.h"
  "../msg_gen/cpp/include/height_quad/EstimateMulti.h"
  "../msg_gen/cpp/include/height_quad/state.h"
  "../msg_gen/cpp/include/height_quad/Attitude.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
