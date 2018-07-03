#include "viewer.h"

void Viewer::draw(){

    glDisable(GL_LIGHTING);

    glPointSize(1);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
    glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glDrawArrays(GL_POINTS, 0, numPoints);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void Viewer::init(){
    restoreStateFromFile();
    help();
    drawCam();
    setPointCloud();
}

QString Viewer::helpString() const {
    QString text("<h2>V i e w e r</h2>");
    text += "Test";
    return text;
}

void Viewer::setPointCloud(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/aisl/catkin_ws/pcd_data.pcd", *cloud); 

    *cloud_filtered = *cloud;

    removeOutlier(cloud_filtered);
    removeDepth(cloud_filtered);
    orientation(cloud_filtered);
    removeOrExtractSurface(cloud_filtered, true);//remove surcafe
    translation(cloud_filtered);

    numPoints = cloud_filtered->points.size();
    MyVertex* tmpBuffer = new MyVertex[numPoints];
    for(int i=0;i<numPoints;i++){
        tmpBuffer[i].point[0] = cloud_filtered->points[i].x;
        tmpBuffer[i].point[1] = cloud_filtered->points[i].y;
        tmpBuffer[i].point[2] = cloud_filtered->points[i].z;
        tmpBuffer[i].color[0] = 200;
        tmpBuffer[i].color[1] = 200;
        tmpBuffer[i].color[2] = 200;
        tmpBuffer[i].color[3] = 200;

    }
    vertexBufferId=0;
    glGenBuffers(1, &vertexBufferId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
    glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * numPoints, tmpBuffer, GL_STATIC_DRAW);
}

void Viewer::drawCam() {
    glPushMatrix();
    glColor3f(1.0,0,0);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0.1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0.1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,0.1);
    glEnd();
    glPopMatrix();

}

void Viewer::removeOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (5);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud);
}

void Viewer::removeDepth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PassThrough<pcl::PointXYZ> pass; 
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y"); 
    pass.setFilterLimits (0.0, 0.3);
    pass.filter (*cloud);
}

void Viewer::removeOrExtractSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool negative){

    //Detect surface
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers, *coefficients);

    //Remove or Extract surface
    //True means removing surface, false means extracting surface
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud);
}

void Viewer::translation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud, xyz_centroid);//Compute centroid of pointcloud

    //Move origin to centroid of pointcloud
    for(size_t i = 0; i < cloud->points.size(); i++){
        cloud->points[i].x = cloud->points[i].x - xyz_centroid[0];
        cloud->points[i].y = cloud->points[i].y - xyz_centroid[1];
        cloud->points[i].z = cloud->points[i].z - xyz_centroid[2];
    }
}


void Viewer::orientation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    removeOrExtractSurface(cloud, false);//Extract surface


}


void estimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
}
