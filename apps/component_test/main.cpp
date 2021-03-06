#include <iostream>
#include <sstream>
#include <iomanip>
#include <common/Defenitions.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/FovisInterface.h>
#include <interfaces/OctomapInterface.h>
#include <interfaces/RRTPlanner.h>
#include <common/PCLUtils.h>
#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

//#include <pcl/registration/icp.h>

#include <QImage>

using namespace KSRobot;
using namespace common;
using namespace interfaces;
using namespace std;

void SaveMapToImage(OccupancyMap::ConstPtr occ, const std::string& filename)
{
    QImage img(occ->Width, occ->Height, QImage::Format_RGB32);
    
    for(size_t i = 0; i < occ->Height; i++)
        for(size_t j = 0; j < occ->Width; j++)
        {
            OccupancyMap::MapPointDataType val = occ->At(i, j);
            QRgb color;
            if( val == OccupancyMap::FreeCell )
            {
                color = qRgb(0, 255, 0);
            }
            else if( val == OccupancyMap::UnknownCell )
            {
                color = qRgb(0, 0, 0);
            }
            else if( val == OccupancyMap::OccupiedCell )
            {
                color = qRgb(0, 0, 255);
            }
            else
            {
                color = qRgb(255, 0, 0);
            }
            img.setPixel(j, i, color);
        }
        
        img.save(filename.c_str());
}

void PlannerTest()
{
    OccupancyMap::Ptr map(new OccupancyMap(400, 400));
    map->Resolution = 1.0f;
    for(int i = -25; i <= 25; i++)
        for(int j = -25; j <= 25; j++)
            map->AtCentered(i, j) = OccupancyMap::OccupiedCell;
    
    for(int i = 25; i <= 75; i++)
        for(int j = -25; j <= 125; j++)
            map->AtCentered(i, j) = OccupancyMap::OccupiedCell;
        
    for(int i = -75; i <= -25; i++)
        for(int j = -25; j <= 125; j++)
            map->AtCentered(i, j) = OccupancyMap::OccupiedCell;
        
    RRTPlanner::Ptr planner(new RRTPlanner);
    ProgramOptions::Ptr po(new ProgramOptions);
    po->LoadFromFile("comptest_settings.xml");
    
    planner->ReadSettings(po->StartNode("PlannerRRT"));
    planner->Initialize(400);
    
    planner->SetOccupancyMap(map);
    planner->SetStartState(Eigen::Vector2f(100, 1), 0);
    planner->SetGoalState(Eigen::Vector2f(-50, -50), 0);
    planner->Plan();
    MotionPlanner::PlannerResult res = planner->Plan();
    std::cout << "Planner result = " << MotionPlanner::GetPlannerResultString(res) << "\n";
    
    if( res == MotionPlanner::SUCCESS )
    {
        MotionPlanner::StateVector vect = planner->GetPlan();
        for(size_t i = 0; i < vect.size(); i++)
        {
            cout << "State " << i << " Pose = (" << vect[i].Position[0] << ", " << vect[i].Position[1] << ") YAW = " << vect[i].Yaw << "\n";
            map->AtCentered((int)vect[i].Position[1], (int)vect[i].Position[0]) = OccupancyMap::FreeCell;
        }
    }

    SaveMapToImage(map, "/home/kourosh/test.png");
    po->SaveToFile("comptest_settings.xml");
}

int main(int argc, char** argv)
{
    (void)argc; (void)argv;
//     PlannerTest();
//     return 0;
    ProgramOptions::Ptr po(new ProgramOptions());
    po->LoadFromFile("comptest_settings.xml");
    
    KinectDatasetReader::Ptr kinect(new KinectDatasetReader);
    FovisInterface::Ptr fovis(new FovisInterface);
    OctomapInterface::Ptr map(new OctomapInterface);
    
    kinect->ReadSettings(po->StartNode("Kinect"));
    fovis->ReadSettings(po->StartNode("Fovis"));
    map->ReadSettings(po->StartNode("Octomap"));
    
    std::string database_dir("/home/kourosh/test/autcup/final/");
    std::string output_dir("/home/kourosh/test/autcup/final/");
    
    kinect->Initialize(database_dir);
    
    fovis->RegisterToKinect(kinect);
    map->RegisterToVO(fovis);

    KinectPointCloud::Ptr final_pc(new KinectPointCloud);
    KinectPointCloud::Ptr tmp_pc1(new KinectPointCloud), tmp_pc2(new KinectPointCloud);

    Eigen::Matrix4f trans;
    trans.setIdentity();
    
    int cycle = 0;
    
    int maxCycles = po->GetInt("MaxCycles", 10);
    
    while( kinect->RunSingleCycle() )
    {
        fovis->RunSingleCycle();
        map->RunSingleCycle();
        
        if( cycle > maxCycles )
            break;
        
//         if( fovis->IsThisCycleKeyframe() )
//         {
//             std::cout << "KEYFRAME\n";
//         }
        
        if( !fovis->Converged() && cycle > 1000 )
        {
            break;
            std::cout << "Failed to converge: rgb file " << kinect->GetCurrentRgbFileName() << std::endl;
            continue;
        }
        KinectPointCloud::ConstPtr curr_pc = kinect->GetPointCloud();
        PCLUtils::ApplyVoxelGrid(*tmp_pc1, curr_pc, 0.02);
        PCLUtils::ApplyPassThrough(*tmp_pc2, tmp_pc1, "z", 0.8f, 4.0f);
        
        pcl::transformPointCloud(*tmp_pc2, *tmp_pc2, fovis->GetGlobalPose());
        final_pc->insert(final_pc->end(), tmp_pc2->begin(), tmp_pc2->end());
        
        Eigen::Isometry3f t = fovis->GetGlobalPose();
        
        std::cout << cycle++  << "\tx =" << t.translation()[0] << " z = " << t.translation()[2] << 
                    " YAW  = " << asinf(-t(2,0)) * 180 / M_PI << " inliers = " << fovis->mFovis->getMotionEstimator()->getNumInliers() << std::endl;
        
        //std::cout << final_pc->size() << "   " << tmp_pc2->size() << std::endl;
        
//         std::stringstream ss2;
//         ss << output_dir << "fpc.pcd";
//         pcl::io::savePCDFileBinary(ss.str(), *final_pc);
    }

    try
    {
        std::stringstream ss;
        std::cout << "Writing final_pointcloud\n" << std::flush;
        ss << output_dir << "final_pointcloud.pcd";
        pcl::io::savePCDFile(ss.str(), *final_pc);
    }
    catch(std::exception& ex)
    {
        std::cout << "Exception in saving pointcloud: " << ex.what() << std::endl;
    }
    catch(...)
    {
        std::cout << "Unknown exception in saving pointcloud.\n";
    }
    
    map->SaveToFile("/home/kourosh/mymap.ot");
    po->SaveToFile("comptest_settings.xml");
    
    // now time for frontier
    OccupancyMap::Ptr occ(new OccupancyMap(100, 100));
    map->ConvertToOccupancyGrid(occ, 50, 50, false);
    //vector<OccupancyMap::Frontier> frs;
    //occ->ExtractFrontiers(frs);
    SaveMapToImage(occ, "/home/kourosh/image.png");
    
    return 0;
}

// int main(int argc, char** argv)
// {
//     (void)argc; (void)argv;
// 
//     KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
//     kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
// 
//     FovisInterface::Ptr fovis(new FovisInterface());
//     fovis->RegisterToKinect(kinect);
// 
//     ProgramOptions::Ptr po(new ProgramOptions);
//     po->LoadFromFile("component-test-settings.xml");
//     float angle = po->GetDouble("AxisRotAngle", 10);
//     angle *= M_PI / 180;
//     
//     Eigen::Quaternionf quatAxis(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()));
//     
//     
//     OctomapInterface::Ptr map(new OctomapInterface);
//     map->RegisterToVO(fovis);
//     map->SetApplyVoxelGrid(true);
//     map->SetVoxelGridResolution(0.05);
//     map->SetMapResolution(0.05);
//     map->SetMaxRange(4);
//     map->Initialize();
//     
//     pcl::VoxelGrid<KinectPointCloud::PointType> vg;
//     vg.setLeafSize(0.05, 0.05, 0.05);
//     
//     pcl::PassThrough<KinectPointCloud::PointType> pass;
//     pass.setFilterFieldName("z");
//     pass.setFilterLimits(0, 4);
// 
//     const size_t max_count = 51;
//     
//     int curr_file_index = 0;
//     int skip_counter = 0;
//     const int skip_max = 5;
//     
//     KinectPointCloud final_pc;
//     
//     for(size_t i = 0; i < max_count; i++)
//     {
//         bool shouldskip;
//         
//         if( ++skip_counter == skip_max )
//         {
//             //DO NOT SKIP! READ THE FILE
//             kinect->SetReadFiles(true);
//             shouldskip = false;
//             skip_counter = 0;
//             curr_file_index++;
//         }
//         else
//         {
//             // SKIP CYCLE
//             kinect->SetReadFiles(false);
//             shouldskip = true;
//         }
//         kinect->RunSingleCycle();
//         
//         if( shouldskip )
//             continue;
//         
//         fovis->RunSingleCycle();
//         
//         Eigen::Vector3f trans = fovis->GetGlobalPose().translation();
//         Eigen::Quaternionf quat(fovis->GetGlobalPose().linear());
//         
//         KinectPointCloud::Ptr p1(new KinectPointCloud);
//         vg.setInputCloud(kinect->GetPointCloud());
//         vg.filter(*p1);
//         
//         pass.setInputCloud(p1);
//         pass.filter(*p1);
//         
//         pcl::transformPointCloud(*p1, *p1, trans, quat);
//     
//         pcl::transformPointCloud(*p1, *p1, Eigen::Vector3f(0,0,0), quatAxis);
//         
//         final_pc.insert(final_pc.end(), p1->begin(), p1->end());
//         
//         map->RunSingleCycle();
//         
//         std::cout << "Cycle " << i << std::endl;
//     }
//     
//     pcl::io::savePCDFileBinaryCompressed("/home/kourosh/test/pointclouds/fovis.pcd", final_pc);
//     po->SaveToFile("component-test-settings.xml");
//     
//     map->SaveToFile("/home/kourosh/map.ot");
//     
// //     KinectDatasetReader::Ptr kinect(new KinectDatasetReader());
// //     kinect->Initialize("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/");
// // 
// //     pcl::VoxelGrid<KinectPointCloud::PointType> vg;
// //     vg.setLeafSize(0.05, 0.05, 0.05);
// //     
// //     pcl::PassThrough<KinectPointCloud::PointType> pass;
// //     pass.setFilterFieldName("z");
// //     pass.setFilterLimits(0, 4);
// // 
// //     const size_t max_count = 2000;
// //     
// //     int curr_file_index = 0;
// //     int skip_counter = 0;
// //     const int skip_max = 5;
// //     
// //     for(size_t i = 0; i < max_count; i++)
// //     {
// //         bool shouldskip;
// //         
// //         if( ++skip_counter == skip_max )
// //         {
// //             //DO NOT SKIP! READ THE FILE
// //             kinect->SetReadFiles(true);
// //             shouldskip = false;
// //             skip_counter = 0;
// //             curr_file_index++;
// //         }
// //         else
// //         {
// //             // SKIP CYCLE
// //             kinect->SetReadFiles(false);
// //             shouldskip = true;
// //         }
// //         kinect->RunSingleCycle();
// //         
// //         if( shouldskip )
// //             continue;
// //         
// //         Eigen::Isometry3f iso = kinect->GetCurrentGroundTruth();
// //         
// //         Eigen::Quaternionf quat(iso.rotation());
// //         Eigen::Vector3f t = iso.translation();
// //         
// //         KinectPointCloud::Ptr p1(new KinectPointCloud);
// //         vg.setInputCloud(kinect->GetPointCloud());
// //         vg.filter(*p1);
// //         
// //         pass.setInputCloud(p1);
// //         pass.filter(*p1);
// //         
// //         p1->sensor_origin_ = Eigen::Vector4f(t[0], t[1], t[2], 0);
// //         p1->sensor_orientation_ = quat;
// //         
// //         pcl::io::savePCDFileBinaryCompressed("/home/kourosh/test/pointclouds/" +
// //             boost::lexical_cast<std::string>(curr_file_index) + ".pcd", *p1);
// //         
// //         //if( i % 10 == 0 || i == max_count - 1 )
// //             std::cout << "Cycle " << i << std::endl;
// //     }
//     
//     
// 
// //     KinectPointCloud pc;
// //     
// //     pcl::VoxelGrid<KinectPointCloud::PointType> vg;
// //     vg.setLeafSize(0.05, 0.05, 0.05);
// //     
// //     pcl::PassThrough<KinectPointCloud::PointType> pass;
// //     pass.setFilterFieldName("z");
// //     pass.setFilterLimits(0, 4);
// //     
// //     const size_t max_count = 50;
// //     
// //     for(size_t i = 1; i < max_count; i++)
// //     {
// //         KinectPointCloud::Ptr loaddedPC(new KinectPointCloud);
// //         pcl::io::loadPCDFile("/home/kourosh/test/pointclouds/" +
// //             boost::lexical_cast<std::string>(i) + ".pcd", *loaddedPC);
// //         
// //         KinectPointCloud p1;
// //         vg.setInputCloud(loaddedPC);
// //         vg.filter(p1);
// //         
// //         *loaddedPC = p1;
// //         
// //         pass.setInputCloud(loaddedPC);
// //         p1.clear();
// //         pass.filter(p1);
// //         
// //         pcl::transformPointCloud(p1, p1, Eigen::Vector3f(p1.sensor_origin_[0], p1.sensor_origin_[1], p1.sensor_origin_[2]),
// //             p1.sensor_orientation_);
// //         
// //         pc.insert(pc.end(), p1.begin(), p1.end());
// //         if( i % 10 == 0 || i == max_count - 1 )
// //             std::cout << "Cycle " << i << std::endl;
// //     }
// //     
// //     pcl::io::savePCDFileBinary("/home/kourosh/test/pointclouds/final.pcd", pc);
//     
//     return 0;
// }
