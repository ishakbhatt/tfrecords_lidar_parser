#ifndef LIDAR_DECODER_HPP 
#define LIDAR_DECODER_HPP

#include "tfrecords_parser.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include "waymo_open_dataset/protos/compressed_lidar.pb.h"

#define DEBUG true

/*
* lidar_decoder.cpp/hpp takes the vector of frames 
* and decodes them into usable lidar messages.
*/

class LidarDecoder 
{

private:
    vector<waymo::open_dataset::CompressedFrameLaserData *> laser_frames;
    std::string & decompressed_data;
    std::vector<Eigen::Vector3D>& points;

public:
    LidarDecoder();
    void decode_raw_data(TFRecordParser& parser);
    ~LidarDecoder();
};

#endif // LIDAR_DECODER_HPP