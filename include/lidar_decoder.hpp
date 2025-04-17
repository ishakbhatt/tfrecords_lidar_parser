#ifndef LIDAR_DECODER_HPP 
#define LIDAR_DECODER_HPP

#include "tfrecords_parser.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <zlib.h>
#include "waymo_open_dataset/protos/compressed_lidar.pb.h"

#include <Eigen/Dense>

#define TOP_LIDAR 1
#define DEBUG true

using PointCloudPtr = std::shared_ptr<std::vector<Eigen::Vector3d>>;

/*
* lidar_decoder.cpp/hpp takes the vector of frames 
* and decodes them into usable lidar messages.
* 
* Laser being decoded is `waymo::open_dataset::LaserName::TOP`
*/

class LidarDecoder 
{

private:

    std::vector<std::vector<std::array<float, 4>>> decompress_laser_data(std::string & compressed_data);
    void bytes_to_points();

    std::vector<waymo::open_dataset::CompressedFrameLaserData *> laser_frames;
    std::string decompressed_data;
    PointCloudPtr points_ptr;

    size_t H;
    size_t W;

public:
    LidarDecoder(PointCloudPtr points_ptr);
    void decode_raw_data(TFRecordParser& parser);
    PointCloudPtr get_lidar_points();
    ~LidarDecoder();
};

#endif // LIDAR_DECODER_HPP
