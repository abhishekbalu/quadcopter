file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/height_quad/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/floor_checker.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
