file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/px_comm/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang)
  include(CMakeFiles/test-results-run.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
