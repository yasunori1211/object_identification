#include<stdio.h>

#include "viewer.h"
// ROS includes 
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
Viewer viewer;

void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT>::Ptr outputPclCloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud);

    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();
    viewer.setPointCloud(cloud, outputPclCloud);
    end = std::chrono::system_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

    outputPclCloud->header.frame_id = input->header.frame_id;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*outputPclCloud, output);
    pub.publish(output);
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "ssh_o");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, pointcloudCb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("ssh_object_identification/output", 1);

    //Recongiture
    dynamic_reconfigure::Server<ssh_object_identification::sshObjectIdentificationParamsConfig> server;
    dynamic_reconfigure::Server<ssh_object_identification::sshObjectIdentificationParamsConfig>::CallbackType f;

    f = boost::bind(&Viewer::dyconCB, &viewer, _1, _2);
    server.setCallback(f);
    
    // Spin
    ros::spin();
}
