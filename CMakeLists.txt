cmake_minimum_required(VERSION 3.8)

project(defrag_optimizer
  LANGUAGES CXX)

# Always produce ANSI-colored output (GNU/Clang only)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  add_compile_options(-fdiagnostics-color=always)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_compile_options(-fcolor-diagnostics)
endif()

add_compile_options(-std=c++17)
add_compile_options(
  -Wall
  -Wextra
  -pedantic-errors)
add_compile_options(
  -Wshadow
  $<$<CXX_COMPILER_ID:Clang>:-Wunreachable-code-return>
  $<$<CXX_COMPILER_ID:GNU>:-Wunreachable-code>)

add_subdirectory(src)
