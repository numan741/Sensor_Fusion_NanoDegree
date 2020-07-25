// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);
    // Initialization of VoxelBox for urilization in Filtering
    pcl::VoxelGrid<PointT> VixelBox;
    VixelBox.setInputCloud (cloud);
    VixelBox.setLeafSize (filterRes, filterRes, filterRes);
    VixelBox.filter (*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr CloudedRegion (new pcl::PointCloud<PointT>);
    // Set the parameter for the region to crop it out.
    pcl::CropBox<PointT> place(true);
    // Setting min and max for the region that is needed
    place.setMin(minPoint);
    place.setMax(maxPoint);
    place.setInputCloud(filteredCloud);
    place.filter(*CloudedRegion);

     
    // Configuration for the roof points removal
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(CloudedRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    for (int i: indices)
        inliers->indices.push_back(i);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (CloudedRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*CloudedRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return CloudedRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
  for(int index:inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
  pcl::SACSegmentation<PointT> seg;
   pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
	seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  
  seg.setInputCloud(cloud);
  seg.segment(*inliers,*coefficients);
  if(inliers->indices.size()==0)
  {
  	std::cout<<"Chould not estimate a planar model for the given dataset"<<std::endl;
  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


/*For segmentation user defined function is defined by using the same logic as implemented in the ransac quiz
//Input is cloud and retured the obstacle and road cloud seprated*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment_Use_RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process as already studied in the course
  auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
  //Initialize the set to accomudate the point that is named as inliers.
   std::unordered_set<int>  inliersOutput;
	srand(time(NULL));
	// For max iterations  to repeat for the correct (approximate) regression in 3d
  while(maxIterations--)
  {
  	std::unordered_set<int> inliers;
    while(inliers.size()<3)
      inliers.insert(rand()%(cloud->points.size()));
    //Initializeation of three points
    float x1,y1,z1,x2,y2,z2,x3,y3,z3;
    auto itr=inliers.begin();
    //getting first point
    x1=cloud->points[*itr].x;
    y1=cloud->points[*itr].y;
    z1=cloud->points[*itr].z;
    
    itr++;
    //Getting seconf point
    x2=cloud->points[*itr].x;
    y2=cloud->points[*itr].y;
    z2=cloud->points[*itr].z;
    
    itr++;
    //Getting third point
    x3=cloud->points[*itr].x;
    y3=cloud->points[*itr].y;
    z3=cloud->points[*itr].z;
    
    
    //Calculating the parameters for the plance->(A,B,C,D)
    float a=((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
    float b=((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)); 
    float c=((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
    float d=-((a * x1) + (b * y1) + (c * z1));
    //iterate fro all the point in the cloud
    //calculating the distance and evaluate with the given distance threshold
    for(int index=0; index<cloud->points.size(); index++){
    	if(inliers.count(index)>0)
          continue;
      	pcl::PointXYZI point =cloud->points[index];
      	float x4 = point.x;
      	float y4 = point.y;
      	float z4 = point.z;
      	float dis = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
      	
      	if(dis<=distanceThreshold)
          inliers.insert(index);
    }
     if(inliers.size()>inliersOutput.size())
      {
      	inliersOutput=inliers;
       
      }
  }

     
     //RansacPlane(cloud, 100, 0.5);
  if(inliersOutput.size()==0)
  {
  	std::cout<<"Chould not estimate a planar model for the given dataset"<<std::endl;
  }
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacle (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    // getting cloud_plane points  and cloud_obstacle
   for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
     //Checking that the point at this index is also in inliers or not(Mean has value or not)
        if(inliersOutput.count(index))
            cloud_plane->points.push_back(point);
        else
            cloud_obstacle->points.push_back(point);
    }


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacle, cloud_plane);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime =std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

   
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  	typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
  	tree->setInputCloud(cloud);
  
  	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance(clusterTolerance);
  	ec.setMinClusterSize(minSize);
  	ec.setMaxClusterSize(maxSize);
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.extract(clusterIndices);
  
  	for(pcl::PointIndices getIndices:clusterIndices){
    	typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
      for(int index:getIndices.indices)
        cloudCluster->points.push_back(cloud->points[index]);
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      
      clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::User_Define_Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
  
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
	//Iniatlization of the Clusters vector
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Initializing kdTree
    KdTree* tree = new KdTree;
    //getting matrix of points
    std::vector<std::vector<float>> points;
    //Interting points in Kd tree data structure.
    for (int i=0; i<cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        
        std::vector<float> point_vector;
        point_vector.push_back(point.x);
        point_vector.push_back(point.y);
        point_vector.push_back(point.z);

        tree->insert(point_vector, i); 
        points.push_back(point_vector);
    }

    // Euclidean Clustering as logic defined in the quiz
    std::vector<std::vector<int>> clusters_idx = euclideanCluster(points, tree, clusterTolerance);
	//Iterate different clusters that we get fron the euclidean clustering
    for(std::vector<int> cluster_idx : clusters_idx)
    {
    	//Defined pointCloud for the specfic type for these searched custers
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice: cluster_idx)
        {
            clusterCloud->points.push_back(cloud->points[indice]);
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        if ((clusterCloud->width >= minSize) and (clusterCloud->width <= maxSize))
            clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}