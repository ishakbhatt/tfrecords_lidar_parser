#include "lidar_decoder.hpp"

LidarDecoder::LidarDecoder()
{
    laser_frames = {};
    decompressed_data = "";
    points = {{}};
}

void LidarDecoder::decode_raw_data(TFRecordParser& parser)
{

    for(auto frame : parser.frames)
    {
        auto * laser_frame = new waymo::open_dataset::CompressedFrameLaserData();
        if(!laser_frame->ParseFromArray(frame->data, frame->record_length))
        {
            std::cerr << "Failed to parse lidar data" << std::endl;;
        }

        if(DEBUG)
        {
            std::cout << "Lasers Size: "<< laser_frame->lasers_size() << std::endl;
        }
        laser_frames.push_back(laser_frame);
    }
}

std::vector<std::vector<std::array<float, 4>>> LidarDecoder::decompress_laser_data(std::string & compressed_data)
{
    size_t compressed_size = compressed_data.size();

    size_t decompressed_size = 1024; // one byte
    std::string decompressed_bytes(decompressed_size, '\0');

    z_stream zs;
    memset(&zs, 0, sizeof(zs));

    // initialize zlib object
    if (inflateInit(&zs) != Z_OK) 
    {
        std::cerr << "Could not initialize zlib object." << std::endl;
    }

    // Set input and output buffers
    zs.next_in = reinterpret_cast<Bytef*>(compressed_data.data());
    zs.avail_in = compressed_size;

    zs.next_out = reinterpret_cast<Bytef*>(decompressed_bytes.data());
    zs.avail_out = decompressed_size;

    // Decompress
    int ret = inflate(&zs, Z_FINISH);
    
    // error checking
    if (ret != Z_STREAM_END && ret != Z_OK) 
    {
        std::cerr << "zlib inflate error: " << ret << std::endl;
        // clean up in case of error
        inflateEnd(&zs);
    }

    // Get the actual decompressed size

    int actual_decompressed_size = decompressed_size - zs.avail_out;

    // clean up zlib
    inflateEnd(&zs);

    const float* decompressed_float_data = reinterpret_cast<const float*>(decompressed_bytes.data());

    std::vector<std::vector<std::array<float, 4>>> decompressed_data(H, std::vector<std::array<float, 4>>(W));


    // decompressed image is of shape: [H][W][4]
    for (int i = 0; i < H; ++i)
    {
        for (int j = 0; j < W; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                decompressed_data[i][j][k] = decompressed_float_data[i * W * 4 + j * 4 + k];
            }
        }
    }

    return decompressed_data;

}


void LidarDecoder::decompressed_data_to_points()
{
    // for every laser scan, convert to cartesian
    for(auto & frame : laser_frames)
    {
        // compressed range data to decompressed
        std::string range_image_delta_compressed = frame->lasers(TOP_LIDAR).ri_return1().range_image_delta_compressed();
        std::vector<std::vector<std::array<float, 4>>> range_image_delta = decompress_laser_data(range_image_delta_compressed); // 2D tensor with 4 values

        // compressed transform to decompressed
        std::string range_image_pose_delta_compressed = frame->lasers(TOP_LIDAR).ri_return1().range_image_pose_delta_compressed();
        std::vector<std::vector<std::array<float, 4>>> range_image_pose = decompress_laser_data(range_image_pose_delta_compressed);

        H = range_image_delta.size();
        W = range_image_delta.at(0).size();

        for(size_t i = 0; i < H; ++i)
        {
            for(size_t j = 0; j < W; ++j)
            {
                // [H][W][range, intensity, elongation, whether range is in no label zone]
                                                                 
                /* Polar coodinates: range, horizontal angle (theta), vertical angle (phi) */
                float range = range_image_delta[i][j][0];
                float theta = j * ((2 * M_PI) / W);
                const auto& calibration = frame->laser_calibrations(i);

                // use min-max-value linear interpolation to compute phi
                double phi_min = calibration.beam_inclination_min();
                double phi_max = calibration.beam_inclination_max();

                /*

                    [Other LIDARS on the vehicle] that have uniform beam inclinations are only parameterized by the min and max.
                    https://github.com/tensorflow/lingvo/blob/master/lingvo/tasks/car/waymo/tools/waymo_proto_to_tfe.py#L454 

                    This enables the choice of doing linear interpolation to compute phi from the min and max inclinations.

                    Calculation:
                    * min value it can be is phi min
                    * each row has a percentage of the height
                    * multiply that by the difference between the min and max

                */
                double phi = phi_min + (i / (H - 1)) * (phi_max - phi_min);

                // spherical to cartesian coordinates
                float x = range * cos(theta) * cos(phi);
                float y = range * cos(theta) * sin(phi);
                float z = range * sin(theta);

                float roll_transform = range_image_pose[i][j][0];
                float pitch_transform = range_image_pose[i][j][1];
                float yaw_transform = range_image_pose[i][j][2];
                float x_transform = range_image_pose[i][j][3];
                float y_transform = range_image_pose[i][j][4];
                float z_transform = range_image_pose[i][j][5];

                Eigen::Vector3d translation(x_transform, y_transform, z_transform);

                // transform x y z from lidar to world coordinates, assuming standard convention
                // https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
                Eigen::AngleAxisd rollAngle(roll_transform, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch_transform, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw_transform, Eigen::Vector3d::UnitZ());

                Eigen::Matrix4d transform_mat = Eigen::Matrix4d::Identity();

                Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

                transform_mat.block<3,3>(0, 0) = rotation;
                transform_mat.block<3,1>(0, 3) = translation;

                // perform a homogeneous transformation (4 x 4 * 4 x 1)
                Eigen::Vector4d point_h(x, y, z, 1.0);

                Eigen::Vector4d transformed_point = transform_mat * point_h;

                // get first 3 values of transformed point
                points.emplace_back(transformed_point.head<3>());

            }
        }

        std::cout << "FRAME" << std::endl;

    }

    std::cout << "Raw lidar ranges to cartesian." << std::endl;

}


LidarDecoder::~LidarDecoder() 
{
    for(auto laser : laser_frames)
    {
        delete laser;
    }
}