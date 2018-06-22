#include<stdio.h>

#include<QtWidgets>
// ROS includes 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h> 

int main(int argv, char* argc[]){
    printf("Hello world!\n");

    return 0;
}
