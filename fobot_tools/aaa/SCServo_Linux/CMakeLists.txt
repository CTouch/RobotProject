cmake_Minimum_required(VERSION 2.8.3)
set(project "SCServo")
project(${project})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB hdrs *.h)
file(GLOB srs *.cpp)

add_library(${project} STATIC ${hdrs} ${srs})

file(GLOB super_feedback_hrds examples/SUPER/FeedBack/*.h)
file(GLOB super_feedback_srs examples/SUPER/FeedBack/*.cc)
add_executable(Super_FeedBack ${super_feedback_hrds} ${super_feedback_srs} ${hdrs} ${srs})
target_link_libraries(Super_FeedBack ncurses pthread)

file(GLOB super_writepos_hrds examples/SUPER/WritePos/*.h)
file(GLOB super_writepos_srs examples/SUPER/WritePos/*.cc)
add_executable(Super_WritePos ${super_writepos_hrds} ${super_writepos_srs} ${hdrs} ${srs})
target_link_libraries(Super_WritePos)

file(GLOB super_gohome_hrds examples/SUPER/GoHome/*.h)
file(GLOB super_gohome_srs examples/SUPER/GoHome/*.cc)
add_executable(Super_GoHome ${super_gohome_hrds} ${super_gohome_srs} ${hdrs} ${srs})
target_link_libraries(Super_GoHome)