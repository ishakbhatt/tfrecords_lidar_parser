#include <google/protobuf/stubs/common.h>
#include "tfrecords_parser.hpp"
#include "lidar_decoder.hpp"
#include "lidar_visualizer.hpp"

using PointCloudPtr = std::shared_ptr<std::vector<Eigen::Vector3d>>;

int main (int argc, char * argv[])
{
    // Verify that the version of the library that we linked against is
    // compatible with the version of the headers we compiled against.
    GOOGLE_PROTOBUF_VERIFY_VERSION;


    if(argc != 2)
    {
        std::cout << "Not enough arguments." << std::endl;        
        return -1;
    }

    // create a single smart pointer for lidar points - decode & visualize
    auto points_ptr = std::make_shared<std::vector<Eigen::Vector3d>>();

    // initialize a parser
    TFRecordParser parser(argv[1]);

    // read records into vector of lidar frame structs
    parser.read_records();

    // raw data to points that can be visualized
    LidarDecoder lidar_decoder(points_ptr);
    lidar_decoder.decode_raw_data(parser);

    // viewer
    LidarVisualizer lidar_visualizer(points_ptr);
    lidar_visualizer.visualize_laser_data();

    return 0;

}