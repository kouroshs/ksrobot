#ifndef KINECTDATASETREADER_H
#define KINECTDATASETREADER_H

#include <common/Defenitions.h>

#include <string>
#include <vector>
#include <exception>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <common/KinectInterface.h>

namespace boost
{
    class thread;
    namespace filesystem
    {
        class path;
    }
};

namespace KSRobot
{
namespace interfaces
{

class KinectDatasetReader : public common::KinectInterface
{
public:
    typedef KinectDatasetReader                 this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    KinectDatasetReader();
    virtual ~KinectDatasetReader();

    virtual void                        Initialize(const std::string& str);
    
    virtual bool                        RunSingleCycle();

    virtual bool                        ProvidesGroundTruth();
    virtual Eigen::Isometry3d           GetCurrentGroundTruth();
    
    int                 GetNumCycles() const { return (int)mRGBFiles.TimeStamps.size(); }
    
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
        Eigen::Isometry3d               LocalTransform;
        double                          TimeStamp;
    };

    
    void                LoadNextFiles();
    
    void                ReadIndexFile           (std::ifstream& file,
                                                 const boost::filesystem::path& fullpath,
                                                 std::vector<double>&   timeStamps,
                                                 std::vector<std::string>& filelist     );
    
    void                ReadGroundTruthData     (std::ifstream& file                    );
    
    void                CorrespondRGBDIndices   (                                       );
    void                CorrespondGroundTruths   (                                       );
    void                MoveGroundTruthsToOrigin(                                       );

private:
    FileList                                    mRGBFiles;
    FileList                                    mDepthFiles;
    
    typedef std::vector<GroundTruthInfo, Eigen::aligned_allocator<GroundTruthInfo> > GroundTruthArray;
    GroundTruthArray                            mGroundTruth;
    
    bool                                        mProvidesGroundTruth;
    int                                         mPerCycleSleep;
    int                                         mTotalTime;
    common::TimePoint                           mLastTime;
    
    volatile bool                               mRunning;
    boost::thread                               mExecThread;
    
    common::Timer::Ptr                          mTimerLoadTimes;
    common::Timer::Ptr                          mTimerPCGenerator;
};

} // end namespace utils
} // end namespace KSRobot

#endif // KINECTDATASETREADER_H
