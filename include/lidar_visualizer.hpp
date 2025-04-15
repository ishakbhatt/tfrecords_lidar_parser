#ifndef LIDAR_VISUALIZER_HPP
#define LIDAR_VISUALIZER_HPP

#include <iostream>
#include <vector>
#include <string>

#include <zlib>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.hpp>

/*
* lidar_visualizer.cpp/hpp takes raw Eigen::Vector3d 
* points and visualizes them in a viewer.
*/

class LidarVisualizer 
{

private:

public:
    LidarVisualizer();

    void visualize_laser_data();

    ~LidarVisualizer();

};

#endif // LIDAR_VISUALIZER_HPP