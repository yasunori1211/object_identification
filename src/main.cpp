#include<stdio.h>
#include<qapplication.h>

//#include<QtWidgets>
#include "viewer.h"
// ROS includes 
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>

Viewer* viewer=0;

void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    viewer->setPointCloud(cloud);
}

void rosThreadLoop( int argc, char** argv )
{
    printf("Started ROS thread\n");

    ros::init(argc, argv, "ssh_object_identification");
    ROS_INFO("Object identification started");

    ros::NodeHandle nh;

    dynamic_reconfigure::Server<ssh_object_identification::ssh_object_identificationConfig> server;
    dynamic_reconfigure::Server<ssh_object_identification::ssh_object_identificationConfig>::CallbackType f;

    f = boost::bind(&Viewer::dyconCB, &*viewer, _1, _2);
    server.setCallback(f);

    ros::Subscriber pointcloud_sub = nh.subscribe(nh.resolveName("camera/depth/points"), 1, pointcloudCb); 

    ros::spin();
    ros::shutdown();
    printf("Exiting ROS thread\n");

    exit(1);
}


int main( int argc, char** argv ) {

    QApplication application(argc, argv);

    viewer = new Viewer();

    viewer->setWindowTitle("viewer");

    viewer->show();


    boost::thread rosThread;

    rosThread = boost::thread(rosThreadLoop, argc, argv);

    application.exec();

    printf("Shutting down... \n");
    ros::shutdown();
    rosThread.join();
    printf("Done. \n");
}
