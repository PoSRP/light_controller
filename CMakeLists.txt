cmake_minimum_required(VERSION 3.8)
project(light_controller CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE "Debug")

add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
