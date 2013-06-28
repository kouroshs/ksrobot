#include <common/ProgramOptions.h>
#include <interfaces/KinectDeviceReader.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <signal.h>


KSRobot::interfaces::KinectDeviceReader::Ptr device;

boost::filesystem::path rgbPath, depthPath;

std::fstream rgbListFile, depthListFile;

volatile bool bContinue = true;

void KinectSaveFn()
{
    static volatile int cycle = 0;
    cycle++;
    std::cout << cycle << std::endl;
    double time = cycle ;
    std::stringstream ss;
    ss << time << ".png";
    
    KSRobot::common::KinectImageDiskIO::SaveToFileRgb((rgbPath / ss.str()).string(), device->GetRgbImage());
    KSRobot::common::KinectImageDiskIO::SaveToFileDepth((depthPath / ss.str()).string(), device->GetRawDepthImage());
    
    //Write to list files
    rgbListFile << time << "  rgb/" << ss.str() << std::endl;
    depthListFile << time << "  depth/" << ss.str() << std::endl;
}

void SigHandler(int signal)
{
    bContinue = false;
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
        
        struct sigaction sigIntHandler; // for Ctrl-C
        sigIntHandler.sa_handler = SigHandler;
        sigemptyset(&(sigIntHandler.sa_mask));
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
        
        rgbListFile.open((basePath / "rgb.txt").string().c_str(), std::ios_base::out);
        depthListFile.open((basePath / "depth.txt").string().c_str(), std::ios_base::out);
        
        device.reset(new KSRobot::interfaces::KinectDeviceReader());
        device->Initialize("");

        device->EnablePointCloudGeneration(false);
        device->EnableFloatDepthGeneration(false);
        
        device->RegisterOnFinishReceiver(KinectSaveFn);
        
        device->Start();
        
        
        while( bContinue )
            boost::this_thread::sleep_for(boost::chrono::seconds(1));
        
        device->Stop();
        
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
