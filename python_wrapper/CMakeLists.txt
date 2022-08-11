cmake_minimum_required(VERSION 3.23)
project(trac_ik)

set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

set(SOURCES
        src/kdl_tl.cpp
        src/nlopt_ik.cpp
        src/trac_ik.cpp
        src/kdl_parser.cpp
        )

SET(URDF_SRCS
        urdf/common.cpp
        urdf/joint.cpp
        urdf/geometry.cpp
        urdf/link.cpp
        urdf/model.cpp
        urdf/tinyxml.cpp
        urdf/tinyxmlerror.cpp
        urdf/tinyxmlparser.cpp
        )

include_directories("${CMAKE_SOURCE_DIR}/cpp/include/automobile_bits")
include_directories("${CMAKE_SOURCE_DIR}/python_wrapper/")


if (WIN32)
    set(Boost_DETAILED_FAILURE_MSG ON)
    set(Boost_THREADAPI win32)
    set(BOOST_ROOT "C:/local/boost_1_79_0")
    set(Boost_LIBRARY_DIR "C:/local/boost_1_79_0/lib64-msvc-14.2")
    set(Boost_COMPILER "-vc142")
    set(Boost_USE_STATIC_RUNTIME ON)
    set(Boost_DEBUG ON)
    set(Boost_ARCHITECTURE "-x64")

    include_directories("D:/chen/lib/nlopt/include"
            "D:/chen/lib/nlopt/bin"
            "D:/chen/lib/eigen-3.4.0/Eigen"
            "D:/chen/lib/kdl/include"
            "C:/local/boost_1_79_0"
            "D:/chen/lib/eigen-3.4.0")
    #    target_link_libraries(trac_ik "/usr/lib/x86_64-linux-gnu/liburdf.so")
#    target_link_libraries(trac_ik "D:/chen/lib/kdl/lib/orocos-kdl.lib")
#    find_package(Boost 1.79.0 REQUIRED COMPONENTS date_time)
#    if (Boost_FOUND)
#        message(" FOUND BOOST: ${Boost_INCLUDE_DIRS}")
#        message("FOUND BOOST: ${Boost_LIBRARIES}")
#    endif (Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIR})
#    target_link_libraries(trac_ik ${Boost_SYSTEM_LIBRARY})
    target_link_libraries(trac_ik "D:/chen/lib/nlopt/lib/nlopt.lib")
    target_link_libraries(trac_ik "D:/chen/lib/kdl/lib/orocos-kdl.lib")
    target_link_libraries(trac_ik "C:/local/boost_1_79_0/lib64-msvc-14.2/libboost_date_time-vc142-mt-s-x64-1_79.lib")
    target_link_libraries(trac_ik "C:/local/boost_1_79_0/lib64-msvc-14.2/libboost_date_time-vc142-mt-s-x64-1_79.lib")
#    add_library(trac_ik "D:/chen/lib/nlopt/bin/nlopt.dll")

endif (WIN32)


if (UNIX)
    find_package(Boost REQUIRED COMPONENTS date_time)
    if (Boost_FOUND)
        message(" FOUND BOOST: ${Boost_INCLUDE_DIRS}")
        message("FOUND BOOST: ${Boost_LIBRARIES}")
    endif (Boost_FOUND)
    include_directories("/usr/include/eigen3"
            "/home/haochen/Desktop/code/kdl_lib/include"
            "/usr/include/"
            "/home/haochen/Desktop/code/nlopt_lib/include"
            ${Boost_INCLUDE_DIRS})
    target_link_libraries(trac_ik "/usr/lib/x86_64-linux-gnu/liburdf.so")
    target_link_libraries(trac_ik "/home/haochen/Desktop/code/kdl_lib/lib/liborocos-kdl.so")
    target_link_libraries(trac_ik "/home/haochen/Desktop/code/nlopt_lib/lib/libnlopt.so")
    target_link_libraries(trac_ik ${Boost_LIBRARIES})
endif (UNIX)


#target_include_directories(src PRIVATE "/usr/include/nlopt.hpp")
#target_include_directories(src PRIVATE "/usr/include/eigen3")
#target_include_directories(src PRIVATE "/home/haochen/Desktop/code/kdl_lib/include/")
#include_directories("/usr/include/SDL2")
#target_link_libraries(src "/usr/lib/x86_64-linux-gnu/libSDL.so")
#add_subdirectory(urdf_parser)


#add_executable(test_boost test/test_boost.cpp test/test_kdl.cpp)
#add_executable(test_eigen test/test_eigen.cpp)
add_executable(test_urdf test/test_urdf.cpp ${URDF_SRCS} ${SOURCES})
target_link_libraries(test_urdf "D:/chen/lib/kdl/lib/orocos-kdl.lib")
target_link_libraries(test_urdf "D:/chen/lib/nlopt/lib/nlopt.lib")
target_link_libraries(test_urdf "D:/chen/lib/kdl/lib/orocos-kdl.lib")
#add_executable(test_kdl test/test_kdl.cpp)
#target_link_libraries(test_kdl "D:/chen/lib/kdl_15/lib/orocos-kdld.lib")

#set(CMAKE_MODULE_PATH "D:/chen/lib/kdl/share/orocos_kdl/cmake;${CMAKE_MODULE_PATH}")
#find_package(orocos_kdl REQUIRED)
#target_link_libraries(test_kdl ${Boost_LIBRARIES})
