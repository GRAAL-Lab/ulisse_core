# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/ulisse_map_node_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/ulisse_map_node_autogen.dir/ParseCache.txt"
  "CMakeFiles/ulissemap_lib_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/ulissemap_lib_autogen.dir/ParseCache.txt"
  "ulisse_map_node_autogen"
  "ulissemap_lib_autogen"
  )
endif()
