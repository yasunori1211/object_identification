#include "objectIdentifier.h"

ObjectIdentifier::ObjectIdentifier(){
    minCylinderRadius = 0.0;
    maxCylinderRadius = 0.1;
    minSphereRadius = 0.0;
    maxSphereRadius = 0.2;
    innerProductThreth = 0.1;
}

void ObjectIdentifier::identifyObject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<PointT>::Ptr output){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //PointXYZ to PointXYZRGB
    pcl::copyPointCloud(*cloud, *cloud_filtered);

    //Remove nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);

    //Down sampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud_filtered);

    //Remove unnecessary point cloud
    removeDepth(cloud_filtered);

    //Remove ground
    detectSurface(cloud_filtered, inliers);
    removeOrExtractSurface(cloud_filtered, inliers, true);

    //Initialize color
    for(int i=0;i<cloud_filtered->points.size();i++){
        cloud_filtered->points[i].r = 200;
        cloud_filtered->points[i].g = 200;
        cloud_filtered->points[i].b = 200;
    }

    //Clustering the point cloud
    clustering(cloud_filtered, clustered_cloud);

    //Identify clustered point cloud
    identify();

    pcl::PointCloud<PointT>::Ptr temp (new pcl::PointCloud<PointT>);
    for(auto cl : clustered_cloud){
        *temp += *cl;
    }

    pcl::copyPointCloud(*temp, *output);
}

void ObjectIdentifier::dyconCB(ssh_object_identification::sshObjectIdentificationParamsConfig &config, uint32_t level){
    std::lock_guard<std::mutex> _guard(mtx);
    minCylinderRadius = config.double_minCylinderRadiusLimits;
    maxCylinderRadius = config.double_maxCylinderRadiusLimits;
    minSphereRadius = config.double_minSphereRadiusLimits;
    maxSphereRadius = config.double_maxSphereRadiusLimits;
    //innerProductThreth = config.double_rectangularInnerProductThreth;
}

void ObjectIdentifier::removeOutlier(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (5);
    sor.setStddevMulThresh (0.1);
    sor.filter (*cloud);
}

void ObjectIdentifier::removeDepth(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::PassThrough<PointT> pass; 
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x"); 
    pass.setFilterLimits (-1.0, 1.0);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y"); 
    pass.setFilterLimits (-1.0, 1.0);
    pass.filter (*cloud);
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*cloud);
}

void ObjectIdentifier::removeOrExtractSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative){
    //Remove or Extract surface
    //True means removing surface, false means extracting surface
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);
    extract.filter(*cloud);
}

void ObjectIdentifier::estimateNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;  
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); 
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*normals);
}

void ObjectIdentifier::identify(){
    std::lock_guard<std::mutex> _guard(mtx);
    /**Identify**/
#pragma omp parallel for
    for(int i=0;i<clustered_cloud.size();i++){
        if(detectCylinder(clustered_cloud[i])){
            std::cout << "Detected cylinder!" << std::endl;
        }else if(detectRectangular(clustered_cloud[i])){
            std::cout << "Detected Rectangular" << std::endl;
        }else if(detectSphere(clustered_cloud[i])){
            std::cout << "Detected Sphere" << std::endl;
        }else{
            std::cout << "Detected Other object" << std::endl;
        }
    }
}

void ObjectIdentifier::detectSurface(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.015);
    seg.segment(*inliers, *coefficients);

    normal = coefficients->values;
}

bool ObjectIdentifier::detectCylinder(pcl::PointCloud<PointT>::Ptr cloud){
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
    seg.setDistanceThreshold(0.03);
    seg.setRadiusLimits(minCylinderRadius, maxCylinderRadius);
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
        return true;
    }else{
        return false; 
    }
}

bool ObjectIdentifier::detectRectangular(pcl::PointCloud<PointT>::Ptr cloud){
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
    for(int i=0;i<coef.size();i++){
        if(i!=coef.size()-1){
            float ip = coef[i][0]*coef[i+1][0] + coef[i][1]*coef[i+1][1] + coef[i][2]*coef[i+1][2]; //inner product( cos(theta) )
            if(std::abs(ip)>innerProductThreth){
                orthogonal=false;
                break;
            }
        }else{
            float ip = coef[i][0]*coef[0][0] + coef[i][1]*coef[0][1] + coef[i][2]*coef[0][2];
            if(std::abs(ip)<innerProductThreth)
                orthogonal=true;
        }
    }

    if(numPlanes>=2){ //Detected rectangular
        if(orthogonal){
            //Add red color to rectangular
            for(int i=0;i<rectangular->points.size();i++){
                rectangular->points[i].r = 255; 
                rectangular->points[i].g = 0; 
                rectangular->points[i].b = 0; 
            }

            *cloud = *rectangular + *cloud_cp;
            return true;
        }
    }else{
        return false; 
    }
}

bool ObjectIdentifier::detectSphere(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> extract;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr sphere(new pcl::PointCloud<PointT>);

    estimateNormal(cloud, cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    //seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.03);
    seg.setRadiusLimits(minSphereRadius, maxSphereRadius);
    seg.setInputNormals (cloud_normals);
    seg.segment(*inliers, *coefficients);

    float ratio = (float)inliers->indices.size()/cloud->points.size();

    if(ratio>=0.5){
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*sphere);
        extract.setInputCloud(cloud);
        extract.setNegative(true);
        extract.filter(*cloud);

        //Adde Color to sphere
        for(int i=0;i<sphere->points.size();i++){
            sphere->points[i].r = 0;
            sphere->points[i].g = 0;
            sphere->points[i].b = 255;
        }

        *cloud += *sphere;
        return true;
    }else{
        return false; 
    }
}

void ObjectIdentifier::clustering(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &clustered_cloud){
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

//    int i,j;
#pragma omp parallel for
    for(int i=0;i<cluster_indices.size();i++){
        pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
        for(int j=0;j<cluster_indices[i].indices.size();j++)
            temp_cloud->points.emplace_back(cloud->points[cluster_indices[i].indices[j]]);

        temp_cloud->width = temp_cloud->points.size();
        temp_cloud->height = 1;
        temp_cloud->is_dense = true;
#pragma omp critical
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
