cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3 CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)

# DBoW2 and g2o from Thirdparty folders
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

# Interface library for common warnings
add_library(orbslam3_warnings INTERFACE)
target_compile_options(orbslam3_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
    -Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:
    /W4>
)

# Optimization/Architecture flags
set(ORBSLAM3_OPTIMIZATION_FLAGS -O3 -march=native) 

# Create shared library
add_library(${PROJECT_NAME} SHARED
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
        src/Atlas.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Viewer.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/OptimizableTypes.cpp
        src/MLPnPsolver.cpp
        src/GeometricTools.cc
        src/TwoViewReconstruction.cc
        src/Config.cc
        src/Settings.cc
)

# Apply warnings and optimization flags
target_link_libraries(${PROJECT_NAME} PRIVATE orbslam3_warnings)
target_compile_options(${PROJECT_NAME} PRIVATE ${ORBSLAM3_OPTIMIZATION_FLAGS})

# Set include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
   ${DBoW2_INCLUDE_DIRS}
   ${g2o_INCLUDE_DIRS}
   ${PROJECT_SOURCE_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
    PUBLIC ${OpenCV_LIBS}
    PUBLIC Eigen3::Eigen
    PUBLIC yaml-cpp::yaml-cpp
    ${Pangolin_LIBRARIES}
    PUBLIC DBoW2
    PUBLIC g2o
    -lboost_serialization
    -lcrypto
)

# Example executables
add_executable(vslamlab_orbslam3_mono Examples/Monocular/vslamlab_orbslam3_mono.cpp)
target_link_libraries(vslamlab_orbslam3_mono PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_rgbd Examples/RGB-D/vslamlab_orbslam3_rgbd.cpp)
target_link_libraries(vslamlab_orbslam3_rgbd PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_stereo Examples/Stereo/vslamlab_orbslam3_stereo.cpp)
target_link_libraries(vslamlab_orbslam3_stereo PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_mono_vi Examples/Monocular-Inertial/vslamlab_orbslam3_mono_vi.cpp)
target_link_libraries(vslamlab_orbslam3_mono_vi PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_stereo_vi Examples/Stereo-Inertial/vslamlab_orbslam3_stereo_vi.cpp)
target_link_libraries(vslamlab_orbslam3_stereo_vi PRIVATE ${PROJECT_NAME})

add_executable(vslamlab_orbslam3_rgbd_vi Examples/RGB-D-Inertial/vslamlab_orbslam3_rgbd_vi.cpp)
target_link_libraries(vslamlab_orbslam3_rgbd_vi PRIVATE ${PROJECT_NAME})

# Install targets and headers
install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_mono
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_rgbd
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_stereo
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_mono_vi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_stereo_vi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS ${PROJECT_NAME} vslamlab_orbslam3_rgbd_vi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)