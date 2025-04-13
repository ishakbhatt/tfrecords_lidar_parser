#include <google/protobuf/stubs/common.h>
#include "tfrecords_parser.hpp"
#include "lidar_decoder.hpp"

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

    TFRecordParser parser(argv[1]);

    parser.read_records();

    LidarDecoder lidar_decoder;

    lidar_decoder.decode_raw_data(parser);

    return 0;

}