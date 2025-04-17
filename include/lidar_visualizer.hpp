#ifndef LIDAR_VISUALIZER_HPP
#define LIDAR_VISUALIZER_HPP

#include <iostream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>

using PointCloudPtr = std::shared_ptr<std::vector<Eigen::Vector3d>>;

/*
* lidar_visualizer.cpp/hpp takes raw Eigen::Vector3d 
* points and visualizes them in a viewer.
*/

class LidarVisualizer 
{

private:
    
    PointCloudPtr points_ptr;

public:
    LidarVisualizer(PointCloudPtr points_ptr);

    void visualize_laser_data();

    ~LidarVisualizer();

};

#endif // LIDAR_VISUALIZER_HPP