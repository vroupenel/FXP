cmake_minimum_required(VERSION 3.3)
project(Hsm)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES src/main.cpp src/hsm.h )
add_executable(main ${SOURCES})


target_link_libraries(main
   ${catkin_LIBRARIES}
 )
