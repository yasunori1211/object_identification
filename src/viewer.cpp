#include "viewer.h"

void Viewer::draw(){
    glDisable(GL_LIGHTING);
    drawCam();
    //drawNormal();

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
    centroid.resize(3);
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

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    *cloud_filtered = *cloud;

    removeOutlier(cloud_filtered);

    calc3DCentroid(cloud_filtered);
    translation(cloud_filtered);

    detectSurface(cloud_filtered, inliers);
    removeOrExtractSurface(cloud_filtered, inliers, true);//remove surface
    orientation(cloud_filtered);

    removeDepth(cloud_filtered);

    calc3DCentroid(cloud_filtered);
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
    glLineWidth(1);
    glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0);
    glVertex3f(0,0,0);
    glVertex3f(0.2,0,0);
    glColor3f(0.0,1.0,0.0);
    glVertex3f(0,0,0);
    glVertex3f(0,0.2,0);
    glColor3f(0.0,0.0,1.0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,0.2);
    glEnd();
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
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (-0.5, 0.0);
    pass.filter (*cloud);
    pass.setFilterFieldName ("x"); 
    pass.setFilterLimits (-0.5, 0.2);
    pass.filter (*cloud);
}

void Viewer::removeOrExtractSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative){
    //Remove or Extract surface
    //True means removing surface, false means extracting surface
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud);
}

void Viewer::translation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    //Move origin to centroid of pointcloud
    for(size_t i = 0; i < cloud->points.size(); i++){
        cloud->points[i].x = cloud->points[i].x - centroid[0];
        cloud->points[i].y = cloud->points[i].y - centroid[1];
        cloud->points[i].z = cloud->points[i].z - centroid[2];
    }
}


void Viewer::orientation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    std::vector<float> origin{0.0,-1.0,0.0};
    float yxCosTheta, yzCosTheta;
    float yxtheta, yztheta;

    float yxscalar = std::sqrt(std::pow(origin[0], 2)+std::pow(origin[1], 2))*std::sqrt(std::pow(normal[0], 2)+std::pow(normal[1], 2));
    float yzscalar = std::sqrt(std::pow(origin[1], 2)+std::pow(origin[2], 2))*std::sqrt(std::pow(normal[1], 2)+std::pow(normal[2], 2));
    yxCosTheta = (origin[0]*normal[0] + origin[1]*normal[1])/yxscalar;
    yzCosTheta = (origin[0]*normal[0] + origin[1]*normal[1])/yzscalar;

    yxtheta = std::acos(yxCosTheta);
    yztheta = std::acos(yzCosTheta);

    Eigen::Matrix3f xOri, zOri, ori;
    Eigen::Vector3f point;
    float xsin, xcos, zsin, zcos;
    xsin = std::sin(-yztheta);
    xcos = std::cos(-yztheta);
    zsin = std::sin(-yxtheta);
    zcos = std::cos(-yxtheta);

    xOri << 1, 0, 0,
            0, xcos, -1*xsin,
            0, xsin, xcos;

    zOri << zcos, -1*zsin, 0,
            zsin, zcos, 0,
            0, 0, 1;

    //ori = zOri*xOri;
    ori = zOri*xOri;

    for(int i=0;i<cloud->points.size();i++){
        point[0] = cloud->points[i].x;
        point[1] = cloud->points[i].y;
        point[2] = cloud->points[i].z;

        point = xOri*point;

        cloud->points[i].x = point[0]; 
        cloud->points[i].y = point[1]; 
        cloud->points[i].z = point[2]; 
    }
}

void Viewer::estimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){ pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*normals);
}

void Viewer::drawNormal(){
    glBegin(GL_LINES);
    glLineWidth(1);
    glColor3f(0.0,0.0,1.0);
    //glVertex3f(centroid[0],centroid[1],centroid[2]);
    //glVertex3f(centroid[0]+normal[0],centroid[1]+normal[1],centroid[2]+normal[2]);
    glVertex3f(0,0,0);
    glVertex3f(normal[0],normal[1],normal[2]);
    glEnd();
}

void Viewer::calc3DCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud, xyz_centroid);

    centroid[0] = xyz_centroid[0];
    centroid[1] = xyz_centroid[1];
    centroid[2] = xyz_centroid[2];
}

void Viewer::detectSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers, *coefficients);

    normal = coefficients->values;
}
