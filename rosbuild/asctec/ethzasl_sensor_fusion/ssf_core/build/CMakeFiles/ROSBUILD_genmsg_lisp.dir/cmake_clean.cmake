FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ssf_core/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/DoubleArrayStamped.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_DoubleArrayStamped.lisp"
  "../msg_gen/lisp/ext_ekf.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ext_ekf.lisp"
  "../msg_gen/lisp/ext_imu.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ext_imu.lisp"
  "../msg_gen/lisp/ext_state.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ext_state.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
