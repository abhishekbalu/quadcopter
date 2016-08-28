FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quad_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/quad_msgs/RaB3DValues.h"
  "../msg_gen/cpp/include/quad_msgs/StateWithCov.h"
  "../msg_gen/cpp/include/quad_msgs/CamDet.h"
  "../msg_gen/cpp/include/quad_msgs/MultiPose.h"
  "../msg_gen/cpp/include/quad_msgs/EstimateSingle.h"
  "../msg_gen/cpp/include/quad_msgs/SinglePose.h"
  "../msg_gen/cpp/include/quad_msgs/Estimate.h"
  "../msg_gen/cpp/include/quad_msgs/RaB3DInfo.h"
  "../msg_gen/cpp/include/quad_msgs/EstimateMulti.h"
  "../msg_gen/cpp/include/quad_msgs/TargetsWithCov.h"
  "../msg_gen/cpp/include/quad_msgs/Targets.h"
  "../msg_gen/cpp/include/quad_msgs/State.h"
  "../msg_gen/cpp/include/quad_msgs/OpticalFlow.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
