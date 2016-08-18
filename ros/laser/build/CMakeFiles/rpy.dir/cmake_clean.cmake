file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/laser/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/rpy.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
