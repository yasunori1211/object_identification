// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h> 
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h> 

#include <cmath>
#include <math.h>
#include <map>
#include <string.h>
#include <mutex>
#include <omp.h>
#include <chrono>

// #include <object_identification/objectIdentificationParamsConfig.h>

typedef pcl::PointXYZRGB PointT;

class ObjectIdentifier{
    public:
        ObjectIdentifier(ros::NodeHandlePtr _nodeHandlePtr);
        void identifyObject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<PointT>::Ptr output); //Main function
        // void dyconCB(object_identification::objectIdentificationParamsConfig &config, uint32_t level); //Reconfigure callback function

    private:
        std::vector<float> normal;
        std::vector<pcl::PointCloud<PointT>::Ptr> clustered_cloud;
        std::mutex mtx;
        ros::NodeHandlePtr nodeHandlePtr;
        ros::Publisher removedDepthPCPublisher;
        ros::Publisher removedSurfacePCPublisher;
        ros::Publisher clusteredPCPublisher;
        bool verbose;

        //parameters for identify
        double minCylinderRadius;
        double maxCylinderRadius;
        double minSphereRadius;
        double maxSphereRadius;
        double innerProductThreth;
        
        //Function Operates Point Cloud
        void removeOrExtractSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);
        void estimateNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
        void removeOutlier(pcl::PointCloud<PointT>::Ptr cloud);
        void removeDepth(pcl::PointCloud<PointT>::Ptr cloud);
        void identify();
        void detectSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers);
        bool detectCylinder(pcl::PointCloud<PointT>::Ptr cloud);
        bool detectSphere(pcl::PointCloud<PointT>::Ptr cloudi);
        bool detectRectangular(pcl::PointCloud<PointT>::Ptr cloud);
        void clustering(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &clustered_cloud);
};

