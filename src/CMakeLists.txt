cmake_minimum_required(VERSION 3.8)

add_executable(defrag_optimizer
  main.cpp
  Node.cpp
  Simulate.cpp)

set_target_properties(defrag_optimizer PROPERTIES
  LINK_FLAGS_RELEASE -s
  PREFIX "")

if(WIN32)
  target_compile_definitions(defrag_optimizer PRIVATE _USE_MATH_DEFINES)
endif()
