#define GL_GLEXT_PROTOTYPES 1
#include <QGLViewer/qglviewer.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h> 
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>

// OpenGL includes
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include <cmath>
#include <math.h>

struct MyVertex
{
    float point[3];
    uchar color[4];
};

class Viewer : public QGLViewer {
    public:
        virtual void draw();
        virtual void init();
        virtual QString helpString() const;
        void setPointCloud();
        void drawCam();
        void removeOrExtractSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);
        void translation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void estimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
        void orientation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void removeDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void drawNormal();
        void calc3DCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void detectSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers);

    private:
        GLuint vertexBufferId;
        int numPoints;
        std::vector<float> normal;
        std::vector<float> centroid;
};

