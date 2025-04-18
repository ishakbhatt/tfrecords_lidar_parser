#include "lidar_decoder.hpp"

LidarDecoder::LidarDecoder(PointCloudPtr points_ptr) : points_ptr(points_ptr)
{
    laser_frames = {};
    decompressed_data = "";
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

    // raw bytes to points
    std::cout << "bruh" << std::endl;
    bytes_to_points();

}

std::vector<std::vector<std::array<float, 4>>> LidarDecoder::decompress_laser_data(std::string & compressed_data)
{
    size_t compressed_size = compressed_data.size();

    size_t decompressed_size = 32768; // one byte
    std::string decompressed_str;
    char decompressed_bytes[decompressed_size];

    z_stream zs;
    memset(&zs, 0, sizeof(zs));

    // initialize zlib object
    if (inflateInit(&zs) != Z_OK) 
    {
        std::cerr << "Could not initialize zlib object." << std::endl;
    }

    int ret = 0;

    // Set input and output buffers
    zs.next_in = reinterpret_cast<Bytef*>(compressed_data.data());
    zs.avail_in = compressed_size;

    do {
        zs.next_out = reinterpret_cast<Bytef*>(decompressed_bytes);
        zs.avail_out = sizeof(decompressed_bytes);

        ret = inflate(&zs, 0);

        if (ret != Z_OK && ret != Z_STREAM_END) {
            inflateEnd(&zs);
            throw std::runtime_error("zlib inflate error: " + std::to_string(ret));
        }

        decompressed_str.append(decompressed_bytes, sizeof(decompressed_bytes) - zs.avail_out);
    } while (ret != Z_STREAM_END);

    // clean up zlib
    inflateEnd(&zs);

    const float* decompressed_float_data = reinterpret_cast<const float*>(decompressed_str.data());

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


void LidarDecoder::bytes_to_points()
{
    // for every laser scan, convert to cartesian
    for(auto & frame : laser_frames)
    {
        std::cout << "bruh2" << std::endl;
        std::cout << "TOP_LIDAR = " << TOP_LIDAR << ", lasers_size = " << frame->lasers_size() << std::endl;

        // compressed range data to decompressed
        std::string range_image_delta_compressed = frame->lasers(TOP_LIDAR).ri_return1().range_image_delta_compressed();
        std::cout << "TOP_LIDAR = " << TOP_LIDAR << ", lasers_size = " << frame->lasers_size() << std::endl;

        std::vector<std::vector<std::array<float, 4>>> range_image_delta = decompress_laser_data(range_image_delta_compressed); // 2D tensor with 4 values

        // compressed transform to decompressed
        std::string range_image_pose_delta_compressed = frame->lasers(TOP_LIDAR).ri_return1().range_image_pose_delta_compressed();
        std::vector<std::vector<std::array<float, 4>>> range_image_pose = decompress_laser_data(range_image_pose_delta_compressed);

        H = range_image_delta.size();
        W = range_image_delta.at(0).size();
        
        std::cout << "bruh2.1" << std::endl;

        for(size_t i = 0; i < H; ++i)
        {
            for(size_t j = 0; j < W; ++j)
            {
                // [H][W][range, intensity, elongation, whether range is in no label zone]
                                                                 
                /* Polar coodinates: range, horizontal angle (theta), vertical angle (phi) */
                float range = range_image_delta[i][j][0];
                std::cout << "bruh2.3" << std::endl;
                float theta = j * ((2 * M_PI) / W);
                const auto& calibration = frame->laser_calibrations(i);
                std::cout << "bruh2.2" << std::endl;

                // use min-max-value linear interpolation to compute phi
                double phi_min = calibration.beam_inclination_min();
                double phi_max = calibration.beam_inclination_max();
                std::cout << "bruh3" << std::endl;

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
                std::cout << "bruh4" << std::endl;

                Eigen::Vector3d translation(x_transform, y_transform, z_transform);

                // transform x y z from lidar to world coordinates, assuming standard convention
                // https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
                Eigen::AngleAxisd rollAngle(roll_transform, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch_transform, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw_transform, Eigen::Vector3d::UnitZ());
                std::cout << "bruh5" << std::endl;

                Eigen::Matrix4d transform_mat = Eigen::Matrix4d::Identity();

                Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

                transform_mat.block<3,3>(0, 0) = rotation;
                transform_mat.block<3,1>(0, 3) = translation;

                // perform a homogeneous transformation (4 x 4 * 4 x 1)
                Eigen::Vector4d point_h(x, y, z, 1.0);

                Eigen::Vector4d transformed_point = transform_mat * point_h;
                std::cout << "bruh6" << std::endl;

                // get first 3 values of transformed point
                points_ptr->emplace_back(transformed_point.head<3>());
                std::cout << "bruh7" << std::endl;

            }
        }

        std::cout << "FRAME" << std::endl;

    }

    std::cout << "Raw lidar ranges to cartesian." << std::endl;

}

PointCloudPtr LidarDecoder::get_lidar_points()
{
    return points_ptr;
}

LidarDecoder::~LidarDecoder() 
{
    for(auto laser : laser_frames)
    {
        delete laser;
    }
}