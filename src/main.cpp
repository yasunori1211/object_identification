#include<stdio.h>

#include "objectIdentifier.h"
// ROS includes 
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
ObjectIdentifier objIdentifier;

void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT>::Ptr outputPclCloud(new pcl::PointCloud<PointT>);
    sensor_msgs::PointCloud2 output;

    //Transform pointclod from ros to pcl
    pcl::fromROSMsg(*input, *cloud);

    //Set pointcloud and Get identified pointcloud
    objIdentifier.identifyObject(cloud, outputPclCloud);

    outputPclCloud->header.frame_id = input->header.frame_id;

    //Transform pointclod from pcl to ros
    pcl::toROSMsg(*outputPclCloud, output);
    
    //Publish identified pointcloud
    pub.publish(output);
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "object_identification");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/kinect2/hd/points", 1, pointcloudCb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("object_identification/output", 1);

    //Recongiture
    dynamic_reconfigure::Server<ssh_object_identification::sshObjectIdentificationParamsConfig> server;
    dynamic_reconfigure::Server<ssh_object_identification::sshObjectIdentificationParamsConfig>::CallbackType f;

    f = boost::bind(&ObjectIdentifier::dyconCB, &objIdentifier, _1, _2);
    server.setCallback(f);
    
    // Spin
    ros::spin();
}
