#include "lidar_decoder.hpp"

LidarDecoder::LidarDecoder() {}

void LidarDecoder::decode_raw_data(TFRecordParser& parser)
{

    for(auto frame : parser.frames)
    {
        waymo::open_dataset::CompressedFrameLaserData lidar_frame;
        if(!lidar_frame.ParseFromArray(frame->data, frame->record_length))
        {
            std::cerr << "Failed to parse lidar data" << std::endl;;
        }

        if(DEBUG)
        {
            std::cout << "Lasers Size: "<< lidar_frame.lasers_size() << std::endl;
        }
    }
}

LidarDecoder::~LidarDecoder() {}