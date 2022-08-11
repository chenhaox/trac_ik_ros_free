cmake_minimum_required(VERSION 3.23)
project(trac_ik)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")

find_package(Boost 1.71.0 REQUIRED COMPONENTS date_time)

if (Boost_FOUND)
    message("FOUND BOOST: ${Boost_INCLUDE_DIRS}")
    message("FOUND BOOST: ${Boost_LIBRARIES}")
endif (Boost_FOUND)

include_directories("/usr/include/eigen3"
        "/home/haochen/Desktop/code/kdl_lib/include"
        "/usr/include/"
        "/home/haochen/Desktop/code/nlopt_lib/include"
        ${Boost_INCLUDE_DIRS})

set(SOURCES
        src/kdl_tl.cpp
        src/nlopt_ik.cpp
        src/trac_ik.cpp
        src/kdl_parser.cpp
        main.cpp
        )

#target_include_directories(src PRIVATE "/usr/include/nlopt.hpp")
#target_include_directories(src PRIVATE "/usr/include/eigen3")
#target_include_directories(src PRIVATE "/home/haochen/Desktop/code/kdl_lib/include/")
#include_directories("/usr/include/SDL2")
#target_link_libraries(src "/usr/lib/x86_64-linux-gnu/libSDL.so")
#add_subdirectory(urdf_parser)

add_executable(trac_ik ${SOURCES})
add_executable(test_boost test/test_boost.cpp)
#add_executable(test_eigen test/test_eigen.cpp)
#add_executable(test_urdf test/test_urdf.cpp)
#target_link_libraries(test_urdf "/usr/lib/x86_64-linux-gnu/liburdf.so")
#target_link_libraries(test_urdf "/home/haochen/Desktop/code/kdl_lib/lib/liborocos-kdl.so")
#target_link_libraries(test_urdf "/home/haochen/Desktop/code/nlopt_lib/lib/libnlopt.so")
#add_executable(src src/src.cpp)
target_link_libraries(trac_ik "/usr/lib/x86_64-linux-gnu/liburdf.so")
target_link_libraries(trac_ik "/home/haochen/Desktop/code/kdl_lib/lib/liborocos-kdl.so")
target_link_libraries(trac_ik "/home/haochen/Desktop/code/nlopt_lib/lib/libnlopt.so")
target_link_libraries(trac_ik ${Boost_LIBRARIES})
#add_executable(test_nlopt test/test_nlopt.cpp)
#target_link_libraries(test_nlopt "/usr/lib/x86_64-linux-gnu/liburdf.so")
#target_link_libraries(test_nlopt "/home/haochen/Desktop/code/kdl_lib/lib/liborocos-kdl.so")
#target_link_libraries(test_nlopt "/home/haochen/Desktop/code/nlopt_lib/lib/libnlopt.so")