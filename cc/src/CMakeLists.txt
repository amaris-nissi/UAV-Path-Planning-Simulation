cmake_minimum_required(VERSION 3.5.0)

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

add_executable(examples
  examples.cc)
target_link_libraries(examples 
  lib_graph 
  lib_environment 
  lib_util)
set_target_properties(examples PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(examples PRIVATE 
  -Wfatal-errors
)

add_executable(path_planning
  path_planning.cc 
  dijkstra2d.h 
  dijkstra2d.cc
  depth_first_search2d.h 
  depth_first_search2d.cc
  a_star2d.h 
  a_star2d.cc
  gnuplot-iostream.h
  gui2d.h
  gui2d.cc
  )
target_link_libraries(path_planning
  lib_graph 
  lib_environment 
  lib_util
  ${Boost_LIBRARIES})
set_target_properties(path_planning PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(path_planning PRIVATE 
  -Wfatal-errors
)

add_executable(polynomial_planning
  polynomial_planning.cc
  )
target_include_directories(polynomial_planning PUBLIC
  ${PROJECT_SOURCE_DIR}/src/P4/src)
target_link_libraries(polynomial_planning
  lib_graph 
  lib_environment 
  lib_util
  lib_p4
  ${Boost_LIBRARIES})
set_target_properties(polynomial_planning PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(polynomial_planning PRIVATE 
  -Wfatal-errors
)

add_executable(full_stack_planning
  full_stack_planning.cc 
  a_star2d.h 
  a_star2d.cc
  gnuplot-iostream.h
  gui2d.h
  gui2d.cc
  )
target_include_directories(full_stack_planning PUBLIC
  ${PROJECT_SOURCE_DIR}/src/P4/src)
target_link_libraries(full_stack_planning
  lib_graph 
  lib_environment 
  lib_util
  lib_p4
  ${Boost_LIBRARIES})
set_target_properties(full_stack_planning PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)
target_compile_options(full_stack_planning PRIVATE 
  -Wfatal-errors
)

