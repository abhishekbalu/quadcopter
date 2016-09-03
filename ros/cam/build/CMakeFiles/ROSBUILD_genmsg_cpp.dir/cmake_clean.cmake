FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/cam/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/cam/imu_acc.h"
  "../msg_gen/cpp/include/cam/QuadPose.h"
  "../msg_gen/cpp/include/cam/state_command.h"
  "../msg_gen/cpp/include/cam/full_state.h"
  "../msg_gen/cpp/include/cam/imu.h"
  "../msg_gen/cpp/include/cam/detections.h"
  "../msg_gen/cpp/include/cam/QuadPoseList.h"
  "../msg_gen/cpp/include/cam/string_stamped.h"
  "../msg_gen/cpp/include/cam/estimate.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
