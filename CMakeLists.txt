cmake_minimum_required(VERSION 3.1)
project(RVWO)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error=unused-parameter -O3")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++14.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

# Modules and packages
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 REQUIRED)
find_package(GLOG REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(TBB REQUIRED)

# Output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include/geometry/
    ${PROJECT_SOURCE_DIR}/include/geometry/camera_models/
    ${PROJECT_SOURCE_DIR}/include/map/
    ${PROJECT_SOURCE_DIR}/include/map/blocks/
    ${PROJECT_SOURCE_DIR}/include/utils/
    ${PROJECT_SOURCE_DIR}/include/utils/drawer/
    ${PROJECT_SOURCE_DIR}/third_party/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    /usr/local/include
    /usr/include
)

# Source files
# Source files
file(GLOB SOURCE_FILES
    "src/map/*.cc"
    "src/map/blocks/*.cc"
    "src/utils/*.cc"
    "src/utils/drawer/*.cc"
    "src/geometry/*.cc"
    "src/geometry/camera_models/*.cc"
    "include/map/*.h"
    "include/map/blocks/*.h"
    "include/utils/*.h"
    "include/utils/drawer/*.h"
    "include/geometry/*.h"
    "include/geometry/camera_models/*.h"
)

# Add library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/third_party/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o.so
    ${GLOG_LIBRARIES}
    -lboost_serialization
    -lcrypto
    -lboost_system
    ${TBB_LIBRARIES}
)

# Examples
# add_executable(rgbd_tum_seg Examples/RGB-D/rgbd_tum_seg.cc)
# target_link_libraries(rgbd_tum_seg ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

# add_executable(stereo_inertial_euroc Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
# target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

# # Add other Stereo-Inertial examples...

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial-W)

# Add other Stereo-Inertial-W examples...

# TUM example
# add_executable(stereo_semantic_openloris Examples/Stereo/stereo_semantic_openloris.cpp)
# target_link_libraries(stereo_semantic_openloris ${PROJECT_NAME})
