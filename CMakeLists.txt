cmake_minimum_required(VERSION 2.8)
project(RELOAD)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
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
        src/Viewer.cc
        src/TcpSocketImageDecoder.cpp
        src/JsonParser.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        /home/toams/lib/boost/lib/
        /home/toams/lib/boost/lib/libboost_serialization.so
        /home/toams/lib/boost/lib/libboost_system.so.1.64.0
        )




set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_executable(os1_run
        src/main.cc)
target_link_libraries(os1_run ${PROJECT_NAME})