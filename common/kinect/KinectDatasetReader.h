#ifndef KINECTDATASETREADER_H
#define KINECTDATASETREADER_H

#include <common/Defenitions.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <exception>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <pcl/io/image_grabber.h>
#include <pcl/common/io.h>

#include <common/kinect/KinectInterface.h>

namespace KSRobot
{
namespace common
{

class KinectDatasetReader : public KinectInterface
{
public:
    KinectDatasetReader(ProgramOptions::Ptr po);
    virtual ~KinectDatasetReader();

    virtual void Initialize(const std::string& str);
    virtual void                        Start();
    virtual void                        Stop();
    virtual bool                        IsRunning();
    virtual boost::signals2::connection RegisterRGBDRawCallback(boost::function<RGBDRawReceiverFn> fn);
    virtual boost::signals2::connection RegisterRGBDFloatCallback(boost::function<RGBDReceiverFn> fn);
    virtual boost::signals2::connection RegisterPointCloudCallback(boost::function<PointCloudReceiverFn> fn);
    
    
//     void                SetPath                 (const std::string& path                );
//     
//     void                GetCurrentFileNames     (std::string& rgb, std::string& depth   ) const;
//     
//     virtual bool                        Initialize();
//     virtual void                        Cleanup();
//     virtual bool                        CaptureFrame();
//     virtual double                      GetTimeStamp();
//     virtual void                        ExportImage(cv::Mat& rgb, cv::Mat& depth);
    
private:
    struct FileList
    {
        std::vector<double>             TimeStamps;
        std::vector<std::string>        FileNames;
        
        void AddData(double time, const std::string name)
        {
            TimeStamps.push_back(time);
            FileNames.push_back(name);
        }
        
        void operator = (const FileList& other)
        {
            TimeStamps = other.TimeStamps;
            FileNames = other.FileNames;
        }
    };
    struct GroundTruthInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Quaternionf              Rotation;
        Eigen::Vector3f                 Position;
        double                          TimeStamp;
    };

    
    void                LoadNextFiles();
    
    void                ReadIndexFile           (std::ifstream& file,
                                                 const boost::filesystem::path& fullpath,
                                                 std::vector<double>&   timeStamps,
                                                 std::vector<std::string>& filelist     );
    
    void                ReadGroundTruthData     (std::ifstream& file,
                                                 std::vector<GroundTruthInfo>& gtInfo   );
    
    void                CorrespondRGBDIndices   (                                       );
    void                CorrespondGroundTruth   (                                       );
    
    //TODO: When finished, what to do?
    void                ThreadEntry             (                                       );
private:
    FileList                                    mRGBFiles;
    FileList                                    mDepthFiles;
    std::vector<GroundTruthInfo>                mGroundTruth;
    
    size_t                                      mCycle;
    
    int                                         mPerCycleSleep;
    int                                         mTotalTime;
    TimePoint                                   mLastTime;
    
    typedef pcl::ImageGrabber<pcl::PointXYZRGBA> ImageGrabber;
    typedef boost::shared_ptr<ImageGrabber>      ImageGrabberPtr;
    
    ImageGrabberPtr                             mPointCloudGenerator;
    KinectPointCloud::Ptr                       mCurrPointCloud;
    
    volatile bool                               mRunning;
    boost::thread                               mExecThread;
    
    KinectRgbImage::Ptr                         mCurrRgb;
    KinectRawDepthImage::Ptr                    mCurrDepthRaw;
    KinectFloatDepthImage::Ptr                       mCurrDepthFloat;

    boost::signals2::signal<PointCloudReceiverFn>            mPointCloudReceivers;
    boost::signals2::signal<RGBDReceiverFn>                  mRGBDReceivers;
    boost::signals2::signal<RGBDRawReceiverFn>               mRGBDRawReceivers;
};

} // end namespace utils
} // end namespace KSRobot

#endif // KINECTDATASETREADER_H
