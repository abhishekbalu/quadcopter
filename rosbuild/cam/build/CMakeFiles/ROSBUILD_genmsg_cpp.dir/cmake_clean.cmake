FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cam/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/cam/QuadPose.h"
  "../msg_gen/cpp/include/cam/detections.h"
  "../msg_gen/cpp/include/cam/QuadPoseList.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
