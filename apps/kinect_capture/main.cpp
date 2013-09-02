#include <common/ProgramOptions.h>
#include <interfaces/KinectDeviceReader.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <signal.h>
#include <tbb/concurrent_queue.h>


//TODO: MAKE THESE OPTIONS, PROBABLY USING ProgramOptions

std::string g_FileExtention = ".png";
int g_SkipCycles = 10;


KSRobot::interfaces::KinectDeviceReader::Ptr device;
boost::filesystem::path rgbPath, depthPath;
std::fstream rgbListFile, depthListFile;
volatile bool bContinue = true;


struct KinectInputData
{
    KSRobot::common::KinectRgbImage::ConstPtr           rgb;
    KSRobot::common::KinectRawDepthImage::ConstPtr      depth;
};

tbb::concurrent_queue<KinectInputData>  kinect_data_queue;

void SaverThread()
{
    static volatile int cycle = 0;
    while( bContinue )
    {
        KinectInputData id;
        while( kinect_data_queue.try_pop(id) )
        {
            cycle++;
            
            if( g_SkipCycles != 0 && cycle % g_SkipCycles != 0 )
                continue;
            
            double time = cycle;
            std::stringstream ss;
            
            ss << time << g_FileExtention;

            KSRobot::common::KinectImageDiskIO::SaveToFileRgb((rgbPath / ss.str()).string(), id.rgb);
            KSRobot::common::KinectImageDiskIO::SaveToFileDepth((depthPath / ss.str()).string(), id.depth);

            rgbListFile << time << "  rgb/" << ss.str() << std::endl;
            depthListFile << time << "  depth/" << ss.str() << std::endl;
            
        }
        //std::cout << "hello!\n" << std::flush;
    }
}

void KinectSaveFn()
{
    KinectInputData id;
    id.rgb = device->GetRgbImage();
    id.depth = device->GetRawDepthImage();
    
    kinect_data_queue.push(id);
    
    static int cycle = 0;
    if( cycle++ % 100 == 0 )
        std::cout << cycle << std::endl << std::flush;
}

void SigHandler(int signal)
{
    (void)signal;
    bContinue = false;
    std::cout << "SIGINT RECIEVED\n" << std::flush;
}

int main(int argc, char** argv)
{
    std::string directory;
    if( argc == 1 )
        directory = "./";
    else if( argc == 2 )
        directory = argv[1];
    else
    {
        std::cout << "Error, should provide path for saving.\n";
        return -1;
    }
    
    try
    {
        boost::filesystem::path basePath(directory);
        rgbPath = basePath / "rgb";
        depthPath = basePath / "depth";
        
        if( !boost::filesystem::exists(basePath) )
            boost::filesystem::create_directory(basePath);
        if( !boost::filesystem::exists(rgbPath) )
            boost::filesystem::create_directory(rgbPath);
        if( !boost::filesystem::exists(depthPath) )
            boost::filesystem::create_directory(depthPath);
        
        
        struct sigaction sigIntHandler; // for Ctrl-C
        sigIntHandler.sa_handler = SigHandler;
        sigemptyset(&(sigIntHandler.sa_mask));
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
        
        rgbListFile.open((basePath / "rgb.txt").string().c_str(), std::ios_base::out);
        depthListFile.open((basePath / "depth.txt").string().c_str(), std::ios_base::out);
        
        device.reset(new KSRobot::interfaces::KinectDeviceReader());
        device->Initialize("");

        std::cout << "base path: " << boost::filesystem::absolute(basePath) << std::endl << std::flush;
        
        device->EnablePointCloudGeneration(false);
        device->EnableFloatDepthGeneration(false);
        
        device->RegisterOnFinishReceiver(KinectSaveFn);
        
        boost::thread saver_thread(SaverThread);
        device->Start();
        
        while( bContinue )
            boost::this_thread::sleep_for(boost::chrono::milliseconds(300));
        
        device->Stop();
        
        saver_thread.join();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100)); // make sure everything is done
        
        rgbListFile << std::flush;
        depthListFile << std::flush;
        
        rgbListFile.close();
        depthListFile.close();
        
    }
    catch(std::exception& ex)
    {
        std::cout << "An exception was cought: " << ex.what() << std::endl;
    }
    
    return 0;
}
