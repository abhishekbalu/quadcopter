FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ssf_core/msg"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/ssf_core/parametersConfig.h"
  "../docs/parametersConfig.dox"
  "../docs/parametersConfig-usage.dox"
  "../src/ssf_core/cfg/parametersConfig.py"
  "../docs/parametersConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
