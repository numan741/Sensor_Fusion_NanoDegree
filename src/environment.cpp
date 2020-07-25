/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  //Initialize the pointProcessorI Object with POintXYZI point
  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //Read the pcd file with pointProcessorI function's loadpcd
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  //Downsampling the loaded pcd with the following parameters
  //VoxelGride parameter 0.35
  //minSize (-35,-5.5,-3,1)
  //maxSize  ( 35,6.5,15, 1)
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.35, Eigen::Vector4f (-35,-5.5,-3,1), Eigen::Vector4f ( 35,6.5,15, 1));
  //Render the downsampled point cloud
  renderPointCloud(viewer,filterCloud,"filterCloud");
  
  
  // Segmentation of loaded(filtered) pcd into two(obstacle and road)
  // Pair of two point Cloud is returned with obstacle and road
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> Segment1 = pointProcessorI->Segment_Use_RANSAC(filterCloud, 100, 0.2);
    
  // Clustering the resulting segment with obstacle to red color anf green to road
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Obsclusters = pointProcessorI->User_Define_Clustering(Segment1.first, 0.8, 10, 5000);
  // render the road points
  renderPointCloud(viewer, Segment1.second, "plane", Color(0,1,0));
  // render obstacle that is clustered in the previous clustering function
  int cluster_id = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: Obsclusters){
    // render cluster point cloud
    renderPointCloud(viewer, cluster, "Obsclusters"+std::to_string(cluster_id), colors[cluster_id]);
    // render Bonding box
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, cluster_id, Color(1,0,0), 1);
    cluster_id++;
  }

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_ost=false;
    bool render_plane=false;
    bool render_cluster=true;
    bool render_box=true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
 Lidar* lidar =new Lidar(cars,0);
    // TODO:: Create point processor

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud=lidar->scan();
  //renderRays(viewer,lidar->position,pointCloud);
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  ProcessPointClouds<pcl::PointXYZ>* pointProcessor=new ProcessPointClouds<pcl::PointXYZ>();
  std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud=pointProcessor->SegmentPlane(inputCloud,100,0.2);
  if(render_ost)
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  if(render_plane)
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));



std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(20,45,0), Color(0,0,1)};

for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
{
    if(render_cluster){
        std::cout << "cluster size ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
    }
    if(render_box){
        Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer,box,clusterId);
    }
      ++clusterId;
}
//renderPointCloud(viewer,segmentCloud.second,"planeCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
  	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	  auto streamIterator = stream.begin();
	  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
        
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
      streamIterator = stream.begin();

      viewer->spinOnce ();
    } 
}