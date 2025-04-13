# Notes
This file includes my personal notes as I learn to build my own 3D object tracker! I will be using the Waymo open dataset.

# Step 1: write a data converter

## The format
* TensorFlow TFRecord and Protocol Buffers (`.proto` files)
* Reasoning: serializing the data (so it can be easily stored / transmitted)

## What I need
* TensorFlow C++ API
* Protobuf
* Waymo files
* Libraries (PCL, Eigen, Open3D, OpenCV)

## Generate classes from .proto files
For more info about protocol buffers (serialization method): https://protobuf.dev/getting-started/cpptutorial/
* `sudo apt install protobuf-compiler -y`
* generate files: `protoc --cpp_out=. waymo_open_dataset/protos/*.proto`

## Read TFRecords 
Goal: read TFRecord messages into a dedicated data structure. 

### Build and link TF (CPU-only C++ runtime)
wget https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-linux-x86_64-2.11.0.tar.gz
sudo tar -C /usr/local -xzf libtensorflow-cpu-linux-x86_64-2.11.0.tar.gz
export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/lib

### Single record format
See [record_writer in tensorflow](https://github.com/tensorflow/tensorflow/blob/516ae286f6cc796e646d14671d94959b129130a4/tensorflow/core/lib/io/record_writer.h#L60).

// Format of a single record:
//  uint64    length
//  uint32    masked crc of length
//  byte      data[length]
//  uint32    masked crc of data

1. Read a TFRecord in binary format
2. Read each into structs, verifying crc's for data corruption
3. Keep doing so until the end of a file

### Data structure to use
* vector of structs: temporal tracking, contiguous, easy to use STL wise
* other options: unordered map of ints and structs (good if there were IDs separate from indices)
* For ease of use we will use a vector of structs

## Decode `data` field in TFRecord
Once the raw data is extracted from the TFRecord, `data` needs to be decoded using the proto classes generated from `protoc`.

This is done with `read_data.hpp/.cpp`.

How we are going to do this:
- Each laser scan has an RI return (Range Image Return, the surface a pulse hits) and there can be many (closest is RI return 1)
- 