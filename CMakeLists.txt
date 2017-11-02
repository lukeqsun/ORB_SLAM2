cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")

set(CMAKE_C_FLAGS_DEBUG "-O2 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g")

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

OPTION(WITH_CUDA "Build with CUDA support" ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

if (WITH_CUDA)
    find_package(CUDA REQUIRED)
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    include_directories(${CUDA_INCLUDE_DIRS})
    file(GLOB cuda_src
      src/cuda/Allocator_gpu.cu
      src/cuda/Fast_gpu.cu
      src/cuda/Orb_gpu.cu
      src/cuda/Cuda.cu
    )
    set(CUDA_ARCH_BIN "30 35 50 52 53 61 62" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
    set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")

    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
    include(CudaComputeTargetFlags.cmake)
    APPEND_TARGET_ARCH_FLAGS()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-std=c++11 -use_fast_math")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
endif()

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(octomap REQUIRED )

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

file(GLOB src
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/Viewer.cc
    src/OctomapBuilder.cc
)

if (WITH_CUDA)
    CUDA_COMPILE(cuda_objs ${cuda_src})
endif()

add_library(${PROJECT_NAME} SHARED
    ${src}
    ${cuda_objs}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${OCTOMAP_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

if (WITH_CUDA)
  target_link_libraries(${PROJECT_NAME}
      ${CUDA_LIBRARIES}
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_CUDA=1)
endif()

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})

if(USE_EXAMPLE_WEBCAM)
    find_package(PkgConfig REQUIRED)
    pkg_search_module(CURL REQUIRED libcurl)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    add_executable(mono_uvc Examples/mono_uvc.cpp)
    target_link_libraries(mono_uvc ${PROJECT_NAME})
    add_executable(mono_android_ipcam Examples/mono_android_ipcam.cpp)
    target_link_libraries(mono_android_ipcam ${PROJECT_NAME} ${CURL_LIBRARIES})
    add_executable(mono_android_ipcam_ar Examples/mono_android_ipcam_ar.cpp Examples/AR/ViewerAR.cc)
    target_include_directories(mono_android_ipcam_ar PUBLIC Examples/AR/)
    target_link_libraries(mono_android_ipcam_ar ${PROJECT_NAME} ${CURL_LIBRARIES})
endif()
