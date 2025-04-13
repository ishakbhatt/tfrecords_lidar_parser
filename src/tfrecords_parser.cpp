#include "tfrecords_parser.hpp"

TFRecordParser::TFRecordParser(std::string file_in)
{
    finput.open(file_in, std::ios::in | std::ios::binary);

    if(!finput.is_open())
    {
        std::cout << "Failed to open tfrecord file." << std::endl;
    }
}


void TFRecordParser::read_records()
{

    uint64_t length = 0;
    char len_buf[8];

    while(finput.read(len_buf, sizeof(length)))
    {
        Frame* frame = new Frame();
        length = *reinterpret_cast<uint64_t*>(len_buf);

        frame->record_length = length;
        std::cout << length << std::endl;
        frame->data = new char[length];

        finput.read(reinterpret_cast<char *>(&frame->length_masked_crc), sizeof(uint32_t));
        finput.read(frame->data, frame->record_length);
        finput.read(reinterpret_cast<char *>(&frame->data_masked_crc), sizeof(uint32_t));

        if(DEBUG)
        {
            std::cout << " ================= FRAME ================= " << std::endl;
            std::cout << "RECORD LENGTH: " << frame->record_length << std::endl;
            std::cout << "LENGTH MASKED CRC: " << frame->length_masked_crc << std::endl;
            std::cout << "DATA MASKED CRC: " << frame->data_masked_crc << std::endl;
        }

        frames.push_back(frame);
    }

    std::cout << "Successfully read frames." << std::endl;
    
}

TFRecordParser::~TFRecordParser()
{
    for(auto frame : frames)
    {
        delete[] frame->data;
        delete frame;
    }
}