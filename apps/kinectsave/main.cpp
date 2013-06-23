
#include <interfaces/KinectDeviceReader.h>
#include <iostream>
#include <sstream>


std::string DefaultPath = "/home/kourosh/kinectsave";
std::string pathToSave;

KinectDeviceReader::Ptr kinect;

void OnKinectReciever()
{
}


int main(int argc, char** argv)
{
    //TODO Add arguments support
    
    pathToSave = DefaultPath;
    
    return 0;
}
