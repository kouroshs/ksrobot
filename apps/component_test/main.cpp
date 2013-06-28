#include <iostream>
#include <sstream>
#include <iomanip>
#include <common/Defenitions.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/FovisInterface.h>
#include <common/MappingInterface.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

using namespace KSRobot;
using namespace common;
using namespace interfaces;
using namespace std;

int main(int argc, char** argv)
{
    ProgramOptions::Ptr settings(new ProgramOptions());
    settings->LoadFromFile("settings.xml");
    
    KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
    FovisInterface::Ptr fovis(new FovisInterface());
    MappingInterface::Ptr map(new MappingInterface());
    
    fovis->ReadSettings(settings->StartNode("FovisInterface"));
    map->ReadSettings(settings->StartNode("MappingInterface"));

    kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
    fovis->RegisterToKinect(kinect);
    //map->RegisterToVO(fovis);

    int maxCount = settings->GetInt("Cycles", 10);
    
    KinectPointCloud::Ptr ki(new KinectPointCloud);
    ki->points.reserve(800 * 640 * 480);
    
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    float sz = 0.01;
    vg.setLeafSize(sz, sz, sz);
    
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 3);
    
    KinectPointCloud::Ptr filterResult(new KinectPointCloud);
    
    std::vector<int> dummyVect;
    size_t numPoints = 0;
    
    for(int i = 0; i < 50; i++)
    {
        kinect->RunSingleCycle();
        fovis->RunSingleCycle();
        //map->RunSingleCycle();
        
        if( !fovis->Converged() )
        {
            std::cout << "SKIPPED\n";
            continue;
        }
        KinectPointCloud::ConstPtr pc = kinect->GetPointCloud();
        vg.setInputCloud(pc);
        filterResult->points.clear();
        vg.filter(*filterResult);
        
        pcl::removeNaNFromPointCloud(*filterResult, *filterResult, dummyVect);
        
        pass.setInputCloud(filterResult);
        pass.filter(*filterResult);
        
        //pcl::transformPointCloud(*filterResult, *filterResult, fovis->GetGlobalPose().cast<float>());
        pcl::transformPointCloud(*filterResult, *filterResult, kinect->GetCurrentGroundTruth().cast<float>());
        ki->points.insert(ki->points.end(), filterResult->begin(), filterResult->end());
        
        Eigen::Vector3d v = fovis->GetGlobalPose().translation();
        
        //if( i % 10 == 0 || i == maxCount - 1 )
        //std::cout << "Cycle " << kinect->GetCycle() << " " << fovis->GetCycle() << " " << map->GetCycle() << std::endl;
        std::cout << v[0] << "  " << v[1] << "  " << v[2] << std::endl;
    }
    settings->SaveToFile("settings.xml");
    //map->SaveMapToFile("/home/kourosh/map.ot");
    
    vg.setInputCloud(ki);
    vg.filter(*ki);
    
    pcl::io::savePCDFileBinary("/home/kourosh/pc.pcd", *ki);
    return 0;
}
