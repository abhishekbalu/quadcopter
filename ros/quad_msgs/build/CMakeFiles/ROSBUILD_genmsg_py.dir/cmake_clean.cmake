FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/quad_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/quad_msgs/msg/__init__.py"
  "../src/quad_msgs/msg/_RaB3DValues.py"
  "../src/quad_msgs/msg/_StateWithCov.py"
  "../src/quad_msgs/msg/_CamDet.py"
  "../src/quad_msgs/msg/_MultiPose.py"
  "../src/quad_msgs/msg/_EstimateSingle.py"
  "../src/quad_msgs/msg/_SinglePose.py"
  "../src/quad_msgs/msg/_Estimate.py"
  "../src/quad_msgs/msg/_RaB3DInfo.py"
  "../src/quad_msgs/msg/_EstimateMulti.py"
  "../src/quad_msgs/msg/_TargetsWithCov.py"
  "../src/quad_msgs/msg/_Targets.py"
  "../src/quad_msgs/msg/_State.py"
  "../src/quad_msgs/msg/_OpticalFlow.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
