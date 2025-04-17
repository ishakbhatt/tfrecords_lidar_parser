cmake_minimum_required(VERSION 3.15...3.25)

project(3d_tracker)
set(CMAKE_CXX_STANDARD 17)

find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization)

file(GLOB WAYMO_PROTO_SRCS waymo-open-dataset/src/waymo_open_dataset/protos/*.pb.cc)
file(GLOB WAYMO_SRCS waymo-open-dataset/src/waymo_open_dataset/*.pb.cc)

# include headers from waymo dataset and include
include_directories(include)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${ZLIB_DEPS_DIR})
include_directories(waymo-open-dataset/src)
include_directories(waymo-open-dataset/src/waymo_open_dataset/protos)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# link sources in a single executable
add_executable(3d_tracker 
    src/main.cpp
    src/lidar_decoder.cpp
    src/lidar_visualizer.cpp
    src/tfrecords_parser.cpp
    ${WAYMO_PROTO_SRCS}
    ${WAYMO_SRCS}
)

# link libraries to executable
target_link_libraries(3d_tracker
    ${Protobuf_LIBRARIES} z ${PCL_LIBRARIES}
)