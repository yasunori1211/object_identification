#include "viewer.h"

void Viewer::init(){
    restoreStateFromFile();
    help();
    setedPC = false;
    numPoints = new int[100];
    glPointSize(5.0);
    glLineWidth(1);
}

void Viewer::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::copyPointCloud(*cloud, *cloud_filtered);

    /**Remove nan points**/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);

    /**Remove ground**/
    detectSurface(cloud_filtered, inliers);
    removeOrExtractSurface(cloud_filtered, inliers, true);//remove surface

    /**Remove unnecessary point cloud**/
    removeDepth(cloud_filtered);

    //Down sampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.02, 0.02, 0.02);
    sor.filter(*cloud_filtered);

    //Initialize color
    for(int i=0;i<cloud_filtered->points.size();i++){
        cloud_filtered->points[i].r = 200;
        cloud_filtered->points[i].g = 200;
        cloud_filtered->points[i].b = 200;
    }

    /**Clustering the point cloud**/
    clustering(cloud_filtered, clustered_cloud);

    identify(cloud_filtered);

    //set some variables for draw
    std::lock_guard<std::mutex> _guard(mtx);
    numCluster = clustered_cloud.size();
    copyPointcloud();
    setNumPoints();

    if(!setedPC)
        setedPC=true;

    update();
}

void Viewer::draw(){
    std::lock_guard<std::mutex> _guard(mtx);
    if(!setedPC)
        return;
    else{
        glDisable(GL_LIGHTING);
        drawOrigin();
        for(int i=0;i<numCluster;i++)
            drawObject(i);
    }
    glColor3f(1.0, 1.0, 1.0);
    drawText(20, 20, "test");
}

void Viewer::drawOrigin() {
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

void Viewer::drawObject(int id){
    glBegin(GL_POINTS);
    for(int i=0;i<numPoints[id];i++){
        glColor3f(result_cloud[id]->points[i].r, result_cloud[id]->points[i].g, result_cloud[id]->points[i].b);
        glVertex3f(result_cloud[id]->points[i].x, result_cloud[id]->points[i].y, result_cloud[id]->points[i].z);
    }
    glEnd();
}

QString Viewer::helpString() const {
    QString text("<h2>V i e w e r</h2>");
    text += "Test";
    return text;
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
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud);
    pass.setFilterFieldName ("x"); 
    pass.setFilterLimits (-1.0, 1.0);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y"); 
    pass.setFilterLimits (-1.0, 1.0);
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

void Viewer::estimateNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;  
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); 
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*normals);

}

void Viewer::calc3DCentroid(pcl::PointCloud<PointT>::Ptr cloud, std::vector<float> &centroid){
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud, xyz_centroid);

    for(int i=0;i<3;i++)
        centroid.emplace_back(xyz_centroid[i]);

}

void Viewer::identify(pcl::PointCloud<PointT>::Ptr cloud){
    /**Identify**/
    for(int i=0;i<clustered_cloud.size();i++)
        if(detectCylinder(clustered_cloud[i], cloud)){
            std::cout << "Detected cylinder!" << std::endl;
        }else if(detectRectangular(clustered_cloud[i], cloud)){
            std::cout << "Detected Rectangular" << std::endl;
        }else{
            std::cout << "Detected Other object" << std::endl;
        }
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

bool Viewer::detectCylinder(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_other){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> extract;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cylinder(new pcl::PointCloud<PointT>);

    estimateNormal(cloud, cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    //seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.5);
    seg.setInputNormals (cloud_normals);
    seg.segment(*inliers, *coefficients);

    float ratio = (float)inliers->indices.size()/cloud->points.size();

    if(ratio>=0.5){
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cylinder);
        extract.setInputCloud(cloud);
        extract.setNegative(true);
        extract.filter(*cloud);

        /**
        //Invert colors of cylider point cloud
        for(int i=0;i<cylinder->points.size();i++){
            cylinder->points[i].r = std::abs(cylinder->points[i].r-255);
            cylinder->points[i].g = std::abs(cylinder->points[i].g-255);
            cylinder->points[i].b = std::abs(cylinder->points[i].b-255);
        }
        **/

        //Adde Color to cylinder
        for(int i=0;i<cylinder->points.size();i++){
            cylinder->points[i].r = 0;
            cylinder->points[i].g = 255;
            cylinder->points[i].b = 0;
        }

        *cloud += *cylinder;
        /**
          pcl::visualization::PCLVisualizer viewer ("3D Viewer");
          viewer.setBackgroundColor(0,0,0);
          pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
          viewer.addPointCloud<PointT> (cloud, rgb, "Input cloud");
          viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"Input cloud");

          while(!viewer.wasStopped())
          {
          viewer.spinOnce (100);
          }
         **/
        return true;
    }else{
        return false; 
    }
}

bool Viewer::detectRectangular(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_other){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> extract;
    pcl::SACSegmentation<PointT> seg;
    pcl::PointCloud<PointT>::Ptr rectangular(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cp(new pcl::PointCloud<PointT>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    *cloud_cp = *cloud;

    int numPlanes=0;
    std::vector<std::vector<float>> coef;
    while(1){
        seg.setInputCloud(cloud_cp);
        seg.segment(*inliers, *coefficients);

        if(inliers->indices.empty())
            break;
        else{
            extract.setInputCloud(cloud_cp);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane);
            extract.setNegative(true);
            extract.filter(*cloud_cp);
            
            *rectangular += *plane;
            coef.emplace_back(coefficients->values);
            numPlanes++;
        }
    }

    bool orthogonal;
    float threshold = 0.1;
    for(int i=0;i<coef.size();i++){
        if(i!=coef.size()-1){
            float ip = coef[i][0]*coef[i+1][0] + coef[i][1]*coef[i+1][1] + coef[i][2]*coef[i+1][2]; //inner product
            if(std::abs(ip)>threshold){
                orthogonal=false;
                break;
            }
        }else{
            float ip = coef[i][0]*coef[0][0] + coef[i][1]*coef[0][1] + coef[i][2]*coef[0][2];
            if(ip<threshold)
                orthogonal=true;
        }
    }

    if(numPlanes>=2&&orthogonal){ //Detected rectangular
        //Add red color to rectangular
        for(int i=0;i<rectangular->points.size();i++){
            rectangular->points[i].r = 255; 
            rectangular->points[i].g = 0; 
            rectangular->points[i].b = 0; 
        }

        *cloud = *rectangular + *cloud_cp;
        /**
          pcl::visualization::PCLVisualizer viewer ("3D Viewer");
          viewer.setBackgroundColor(0,0,0);
          pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
          viewer.addPointCloud<PointT> (cloud, rgb, "Input cloud");
          viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"Input cloud");

          while(!viewer.wasStopped())
          {
          viewer.spinOnce (100);
          }
          **/
        return true;
    }else{
        return false; 
    }
}

void Viewer::clustering(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &clustered_cloud){
    clustered_cloud.clear();
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

    for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it){
        pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
        for(std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end(); pit++)
            temp_cloud->points.push_back(cloud->points[*pit]);
        temp_cloud->width = temp_cloud->points.size();
        temp_cloud->height = 1;
        temp_cloud->is_dense = true;
        clustered_cloud.emplace_back(temp_cloud);
    }

    /**
    //Add color to each cluster
    float colors[6][3] = {{255,0,0},{0,255,0},{0,0,255},{255,255,0},{255,0,255},{0,255,255}};
    int j=0;
    for(auto cl : clustered_cloud){
        for(int i=0;i<cl->points.size();i++){
            cl->points[i].r = colors[j%6][0];
            cl->points[i].g = colors[j%6][1];
            cl->points[i].b = colors[j%6][2];
        }
        j++;
    }
    **/
    std::cout << "Found " << cluster_indices.size() << " cluster" << std::endl;
}

void Viewer::copyPointcloud(){
    result_cloud.clear();
    for(auto cl : clustered_cloud){
        result_cloud.emplace_back(cl);
    }
}

void Viewer::setNumPoints(){
    int i=0;
    for(auto cl : clustered_cloud){
        int tmpNumPoints = cl->points.size();
        numPoints[i] = tmpNumPoints;
        i++;
    }
}

