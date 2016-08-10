FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/laser/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/full_pose.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_full_pose.lisp"
  "../msg_gen/lisp/Attitude.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Attitude.lisp"
  "../msg_gen/lisp/EstimateSingle.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_EstimateSingle.lisp"
  "../msg_gen/lisp/EstimateMulti.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_EstimateMulti.lisp"
  "../msg_gen/lisp/state.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_state.lisp"
  "../msg_gen/lisp/Estimate.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Estimate.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
