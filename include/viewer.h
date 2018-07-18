#define GL_GLEXT_PROTOTYPES 1
#include <QGLViewer/qglviewer.h>
#include <qmessagebox.h>

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
// OpenGL includes
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include <cmath>
#include <math.h>
#include <map>
#include <string.h>
#include <mutex>

typedef pcl::PointXYZRGB PointT;

struct MyVertex
{
    float point[3];
    uchar color[4];
};

struct FramePoints
{
    GLfloat point[3];
};

class Viewer : public QGLViewer {
    public:
        virtual void init();
        void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        virtual void draw();
        void drawOrigin();
        void drawObject(int id);
        virtual QString helpString() const;

        //Function Operates Point Cloud
        void removeOrExtractSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);
        void estimateNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
        void calc3DCentroid(pcl::PointCloud<PointT>::Ptr cloud, std::vector<float> &centroid);
        void removeOutlier(pcl::PointCloud<PointT>::Ptr cloud);
        void removeDepth(pcl::PointCloud<PointT>::Ptr cloud);
        void identify(pcl::PointCloud<PointT>::Ptr cloud);
        void detectSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers);
        bool detectCylinder(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_other);
        bool detectRectangular(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_other);
        void clustering(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &clustered_cloud);
        void setNumPoints();
        void copyPointcloud();

    private:
        bool setedPC;
        int *numPoints;
        int numCluster;
        std::vector<float> normal;
        std::vector<pcl::PointCloud<PointT>::Ptr> clustered_cloud;
        std::vector<pcl::PointCloud<PointT>::Ptr> result_cloud;
        std::vector<std::vector<float>> objCentroid;
        std::mutex mtx;
};

