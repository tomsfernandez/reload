cmake_minimum_required(VERSION 2.8)
project(RELOAD)

set(Boost_DEBUG 1)

if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    conan_basic_setup()
else()
    message(WARNING "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif()

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Pangolin REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/deps/DBoW2/DBoW2
        ${PROJECT_SOURCE_DIR}/deps/DBoW2/DUtils
        ${PROJECT_SOURCE_DIR}/deps/g2o
        ${EIGEN3_INCLUDE_DIR}
        )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/main.cc
        src/PuntoLejano.cc
        src/PuntosLejanosDB.cc
        src/Serializer.cpp
        src/KeyFrameTriangulacion.cc
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
        src/Video.cpp
        src/Viewer.cc deps/g2o/config.h.in)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/deps/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/deps/g2o/build/lib/libg2o.so
        )

add_executable(reload src/main.cc)
target_link_libraries(reload ${PROJECT_NAME})