#ifndef TFRECORDS_PARSER_HPP 
#define TFRECORDS_PARSER_HPP

#include <iostream>
#include <fstream>
#include <vector>
//#include <tensorflow/io_ops>
#include "compressed_lidar.pb.h"

#define DEBUG true

/*
* tf_record_parser.cpp/hpp reads the TFRecords into usable structures.
* A single message can be parsed byte-wise. 
*/

struct Frame
{
    uint64_t record_length;
    uint32_t length_masked_crc;
    char * data;
    uint32_t data_masked_crc;
};

class TFRecordParser 
{

private:
    std::fstream finput;  
    std::vector<Frame *> frames;

    // needs frames
    friend class LidarDecoder; 

public:
    TFRecordParser(std::string file_in);
    void read_records();
    ~TFRecordParser();

};

#endif // TFRECORDS_PARSER_HPP