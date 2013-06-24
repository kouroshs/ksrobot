#include <iostream>
#include <sstream>
#include <iomanip>
#include <common/Defenitions.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/FovisInterface.h>
#include <common/MappingInterface.h>
#include <gui/Utils.h>
//#include <gui/PointCloudViewer.h>

//#include <QApplication>
//#include <QVTKApplication.h>
//#include <QVTKWidget.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>

using namespace KSRobot;
using namespace common;
using namespace gui;
using namespace interfaces;
using namespace std;

int main(int argc, char** argv)
{
    ProgramOptions::Ptr settings(new ProgramOptions());
    settings->LoadFromFile("settings.xml");
    
    KinectDatasetReader::Ptr kinect(new KinectDatasetReader("kinect"));
    FovisInterface::Ptr fovis(new FovisInterface("Fovis"));
    MappingInterface::Ptr map(new MappingInterface("OctoMap"));
    
    fovis->ReadSettings(settings->StartNode("FovisInterface"));
    map->ReadSettings(settings->StartNode("MappingInterface"));

    kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
    fovis->RegisterToKinect(kinect);
// //     map->RegisterToVO(fovis);
//     
    int maxCount = settings->GetInt("Cycles", 10);
    
    for(int i = 0; i < maxCount; i++)
    {
        kinect->RunSingleCycle();
        fovis->RunSingleCycle();
        //map->RunSingleCycle();
        cout << "Cycle " << kinect->GetCycle() << "  " << fovis->GetCycle() <<
            "  " << map->GetCycle() << std::endl;
    }
//     //settings->SaveToFile("settings.xml");
//     
//     map->SaveMapToFile("/home/kourosh/map.ot");
    
 /*   
    kinect->WriteRunningTimes(cout);
    std::cout << "\n\n";
    fovis->WriteRunningTimes(cout);
    std::cout << "\n\n";
    map->WriteRunningTimes(cout);
 */   
//     cin.ignore();
//     
//     dr.RunSingleCycle();
//     
//     KinectPointCloud::ConstPtr pc = dr.GetPointCloud();
// 
//     pcl::io::savePCDFileASCII(string("/home/kourosh/1.pcd"), *pc);
//     
    return 0;
}