# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/q_simulation_interfaces_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/q_simulation_interfaces_autogen.dir/ParseCache.txt"
  "q_simulation_interfaces_autogen"
  )
endif()
