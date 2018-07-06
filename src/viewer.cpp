#include "viewer.h"

void Viewer::draw(){
    glDisable(GL_LIGHTING);
    drawOrigin();
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
    centroid.resize(3);//Atamawarui
    setPointCloud();
}

QString Viewer::helpString() const {
    QString text("<h2>V i e w e r</h2>");
    text += "Test";
    return text;
}

void Viewer::setPointCloud(){
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("/home/aisl/catkin_ws/pcd_data.pcd", *cloud); 

    /**Remove nan points**/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int i=0;i<cloud->points.size();i++){
        cloud->points[i].r = 200;
        cloud->points[i].g = 200;
        cloud->points[i].b = 200;
    }

    *cloud_filtered = *cloud;

    //removeOutlier(cloud_filtered);

    /**Move origin to centroid of point cloud**/
    calc3DCentroid(cloud_filtered);
    translation(cloud_filtered);
    /**Remove ground**/
    detectSurface(cloud_filtered, inliers);
    removeOrExtractSurface(cloud_filtered, inliers, true);//remove surface
    /**Rotate**/
    orientation(cloud_filtered);
    /**Remove unnecessary point cloud**/
    removeDepth(cloud_filtered);
    /**Move again**/
    calc3DCentroid(cloud_filtered);
    translation(cloud_filtered);
    /**Detect cylinder**/
    //*cloud_cylinder = *cloud_filtered;
    //detectCylinder(cloud_cylinder, cloud_filtered);

    std::vector<pcl::PointCloud<PointT>::Ptr> clustered_cloud;
    clustering(cloud_filtered, clustered_cloud);
    /**
    float max, min;
    for(int i=0;i<cloud_filtered->points.size();i++){
        if(i==0){
            max=min=std::abs(cloud_filtered->points[i].y);
        }
        else{
            if(std::abs(cloud_filtered->points[i].y)<min) min = std::abs(cloud_filtered->points[i].y);
            if(std::abs(cloud_filtered->points[i].y)>max) max = std::abs(cloud_filtered->points[i].y);
        }
    }
    max = 0.1;
    for(int i=0;i<cloud_filtered->points.size();i++){
        int color;
        if(std::abs(cloud_filtered->points[i].y)>max) color = 255*(max-min)/(max-min);
        else color = 255*((std::abs(cloud_filtered->points[i].y)-min)/(max-min));
        cloud_filtered->points[i].r = color;
        cloud_filtered->points[i].g = color;
        cloud_filtered->points[i].b = color;
    }
    **/
    if(!cloud_cylinder->points.empty()){
        *cloud_filtered += *cloud_cylinder;
        std::cout << "filtered poitns " << cloud_filtered->points.size() << std::endl;
        std::cout << "cylinder poitns " << cloud_cylinder->points.size() << std::endl;
    }

    numPoints = cloud_filtered->points.size();
    MyVertex* tmpBuffer = new MyVertex[numPoints];
    for(int i=0;i<numPoints;i++){
        tmpBuffer[i].point[0] = cloud_filtered->points[i].x;
        tmpBuffer[i].point[1] = cloud_filtered->points[i].y;
        tmpBuffer[i].point[2] = cloud_filtered->points[i].z;
        tmpBuffer[i].color[0] = cloud_filtered->points[i].r;
        tmpBuffer[i].color[1] = cloud_filtered->points[i].g;
        tmpBuffer[i].color[2] = cloud_filtered->points[i].b;
        tmpBuffer[i].color[3] = 255;
    }
    vertexBufferId=0;
    glGenBuffers(1, &vertexBufferId);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId);
    glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * numPoints, tmpBuffer, GL_STATIC_DRAW);
}

    void Viewer::drawOrigin() {
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

void Viewer::removeOutlier(pcl::PointCloud<PointT>::Ptr cloud){
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (5);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud);
}

void Viewer::removeDepth(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::PassThrough<PointT> pass; 
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (-0.5, 0.0);
    pass.filter (*cloud);
    pass.setFilterFieldName ("x"); 
    pass.setFilterLimits (-0.5, 0.2);
    pass.filter (*cloud);
}

void Viewer::removeOrExtractSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative){
    //Remove or Extract surface
    //True means removing surface, false means extracting surface
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud);
}

void Viewer::translation(pcl::PointCloud<PointT>::Ptr cloud){
    //Move origin to centroid of pointcloud
    for(size_t i = 0; i < cloud->points.size(); i++){
        cloud->points[i].x = cloud->points[i].x - centroid[0];
        cloud->points[i].y = cloud->points[i].y - centroid[1];
        cloud->points[i].z = cloud->points[i].z - centroid[2];
    }
}


void Viewer::orientation(pcl::PointCloud<PointT>::Ptr cloud){
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

void Viewer::estimateNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;  
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); 
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*normals);


    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb, "Input cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"Input cloud");
    viewer.addPointCloudNormals<PointT,pcl::Normal>(cloud,normals,10,0.05,"normals");

    while(!viewer.wasStopped())
    {
        viewer.spinOnce (100);
    }
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

void Viewer::calc3DCentroid(pcl::PointCloud<PointT>::Ptr cloud){
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud, xyz_centroid);

    centroid[0] = xyz_centroid[0];
    centroid[1] = xyz_centroid[1];
    centroid[2] = xyz_centroid[2];
}

void Viewer::detectSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers, *coefficients);

    normal = coefficients->values;
}

void Viewer::detectCylinder(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_other){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> extract;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    estimateNormal(cloud, cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    //seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.2);
    seg.setInputNormals (cloud_normals);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
    extract.setInputCloud(cloud_other);
    extract.setNegative(true);
    extract.filter(*cloud_other);

    for(int i=0;i<cloud->points.size();i++){
        cloud->points[i].r = 255;
        cloud->points[i].g = 0;
        cloud->points[i].b = 0;
    }
}

void Viewer::clustering(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> clustered_cloud){
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);  
    tree->setInputCloud (cloud);  

    std::vector<pcl::PointIndices> cluster_indices;  
    pcl::EuclideanClusterExtraction<PointT> ec;  
    ec.setClusterTolerance(0.02); // 2cm  
    ec.setMinClusterSize(100);  
    ec.setMaxClusterSize(25000);  
    ec.setSearchMethod(tree);  
    ec.setInputCloud(cloud);  
    ec.extract (cluster_indices);  

    float colors[3][3] = {{255,0,0},{0,255,0},{0,0,255}};
    int j=0;
    for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end(); pit++){
            cloud->points[*pit].r = colors[j%3][0];
            cloud->points[*pit].g = colors[j%3][1];
            cloud->points[*pit].b = colors[j%3][2];
        }
        j++;
    }
    std::cout << "Find " << cluster_indices.size() << " cluster" << std::endl;
}
