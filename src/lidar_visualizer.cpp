#include <lidar_visualizer.hpp>


LidarVisualizer::LidarVisualizer()
{

}

LidarVisualizer::~LidarVisualizer()
{

}

void LidarVisualizer::visualize_laser_data()
{

    /*
    * https://pcl.readthedocs.io/projects/tutorials/en/master/pcl_visualizer.html
    *
    */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(const auto& pt : points)
    {
        // add points to cloud with emplace_back (in-place push_back)
        cloud->points.emplace_back(pt.x(), pt.y(), pt.z());
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->initCameraParameters();
    
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);

    viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    
    viewer->addCoordinateSystem(1.0);
  
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);
  
    // Run the visualization
    while (!viewer.wasStopped()) 
    {
        viewer.spinOnce();
    }

    delete cloud;
}