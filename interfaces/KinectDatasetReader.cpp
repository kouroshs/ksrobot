#include <interfaces/KinectDatasetReader.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time.hpp>
#include <cmath>

#include <pcl/point_cloud.h> //basic pcl includes
#include <pcl/point_types.h>

namespace KSRobot
{
namespace interfaces
{

using boost::lexical_cast;
using namespace std;
using namespace Eigen;
namespace fs = boost::filesystem;

KinectDatasetReader::KinectDatasetReader() : KinectInterface(), 
    mRunning(false), mTimerLoadTimes(new common::Timer("Image load times")),
    mTimerPCGenerator(new common::Timer("PointCloud generation time")),
    mReadFiles(true)
{
    RegisterTimer(mTimerLoadTimes);
    RegisterTimer(mTimerPCGenerator);
}

KinectDatasetReader::~KinectDatasetReader()
{
}

#define KINECT_TIME_DIFFERENCE (0.5 / 30.0)
static inline bool IsNear(double diff)
{
    return std::fabs(diff) <= KINECT_TIME_DIFFERENCE;
}

void KinectDatasetReader::Initialize(const std::string& pathStr)
{
    fs::path dirPath(pathStr);
    
    if( dirPath == "" || !fs::exists(dirPath) )
        throw std::runtime_error("(KinectDatasetReader::Initialize) Path does not exist.");
    
    fs::path rgbIndexFile = dirPath / "rgb.txt";
    fs::path depthIndexFile = dirPath / "depth.txt";
    fs::path gtPath = dirPath / "groundtruth.txt";
    
    if( !fs::exists(rgbIndexFile) || !fs::exists(depthIndexFile) )
        throw std::runtime_error("(KinectDatasetReader::Initialize) Invalid dataset path: Index files do not exist." + pathStr);
    
    ifstream depthFile(depthIndexFile.string().c_str(), ios::in);
    ifstream rgbFile(rgbIndexFile.string().c_str(), ios::in);
    
    ReadIndexFile(rgbFile, dirPath, mRGBFiles.TimeStamps, mRGBFiles.FileNames);
    rgbFile.close();
    ReadIndexFile(depthFile, dirPath, mDepthFiles.TimeStamps, mDepthFiles.FileNames);
    depthFile.close();

    // Now read the ground truth data.
    ifstream gtFile(gtPath.string().c_str(), ios::in);
    
    if( gtFile.is_open() )
    {
        ReadGroundTruthData(gtFile);
        mProvidesGroundTruth = true;
    }
    else
        mProvidesGroundTruth = false;
    
    gtFile.close();
    // Now check which indexes correspond together
    CorrespondRGBDIndices();
    // Now check those indexes with ground truth data.
    if( mProvidesGroundTruth )
    {
        CorrespondGroundTruths();
        MoveGroundTruthsToOrigin();
    }
    memset(&mParams, 0, sizeof(mParams));
    mParams.Width = 640;
    mParams.Height = 480;
    mParams.FocalX = 528.49404721f;
    mParams.FocalY = 528.49404721f;
    mParams.CenterX = mParams.Width / 2 - 0.5f;
    mParams.CenterY = mParams.Height / 2 - 0.5f;
}

void KinectDatasetReader::MoveGroundTruthsToOrigin()
{
    Eigen::Isometry3f origin = mGroundTruth[0].LocalTransform;
    origin = origin.inverse();
    
    for(size_t i = 0; i < mGroundTruth.size(); i++)
        mGroundTruth[i].LocalTransform = origin * mGroundTruth[i].LocalTransform;
}

static inline bool IsEmpty(const std::string& val)
{
    return val == "";
}

void KinectDatasetReader::ReadIndexFile(ifstream& file, const fs::path& fullPath,
                                        vector<double>& timeStamps, vector<string>& filelist)
{
    timeStamps.clear();
    filelist.clear();

    std::string line;
    while( !file.eof() )
    {
        std::getline(file, line);
        boost::trim(line);
        // Check to see if this line is a comment
        if( line[0] == '#' || line.length() == 0 )
            continue;
        // Now parse the line, it should be like this:  TimeStamp  PathToFile
        vector<string> strs;
        boost::split(strs, line, boost::is_any_of("\t "));
        
        for(size_t i = 0; i < strs.size(); i++)
            boost::trim(strs[i]);
        // remove empty ones
        strs.erase(std::remove_if(strs.begin(), strs.end(), IsEmpty), strs.end());
        if( strs.size() != 2 )
        {
            throw std::runtime_error("(KinectDatasetReader::ReadIndexFile) File parse error.");
            continue;
        }

        timeStamps.push_back(boost::lexical_cast<double>(strs[0]));
        filelist.push_back((fullPath / strs[1]).string());
    }
}

void KinectDatasetReader::ReadGroundTruthData(ifstream& file)
{
    mGroundTruth.clear();
    string line;
    std::vector<std::string> strs;
    while( !file.eof() )
    {
        std::getline(file, line);
        boost::trim(line);

        if( line[0] == '#' || line.length() == 0 )
            continue;

        strs.clear();
        boost::split(strs, line, boost::is_any_of("\t "));

        if( strs.size() != 8 )
        {
            throw std::runtime_error("Invalid dataset file.");
        }

        GroundTruthInfo gt;
        Eigen::Vector3f vect;
        Eigen::Quaternionf rot;
        
        gt.TimeStamp = lexical_cast<double>(strs[0]);
        
        for(int i = 0; i < 3; i++)
            vect[i] = lexical_cast<double>(strs[i + 1]);

        // in form of wxyz
        rot = Quaternionf(lexical_cast<double>(strs[7]),
                                  lexical_cast<double>(strs[4]),
                                  lexical_cast<double>(strs[5]),
                                  lexical_cast<double>(strs[6]));

        gt.LocalTransform.setIdentity();
        // This ordering is necessary to get correct results.
        //TODO: Why in this order? shouldn't it be in the reverse order?
        gt.LocalTransform.translate(vect);
        gt.LocalTransform.rotate(rot);
        
        mGroundTruth.push_back(gt);
    }
}

void KinectDatasetReader::CorrespondRGBDIndices()
{
    size_t iCurrRGBIndex = 0, iCurrDepthIndex = 0;
    std::vector< std::pair<size_t, size_t> > rgbdList;
    int loopCount = 0;
    while( iCurrRGBIndex < mRGBFiles.TimeStamps.size() &&
            iCurrDepthIndex < mDepthFiles.TimeStamps.size() )
    {
        double currRGBTime = mRGBFiles.TimeStamps[iCurrRGBIndex];
        double currDepthTime = mDepthFiles.TimeStamps[iCurrDepthIndex];

        if( IsNear(currDepthTime - currRGBTime) )
        {
            // For all those that are in the range, select the one closest to the current time frame
            size_t currDepth = iCurrDepthIndex + 1;
            size_t closestDepth = iCurrDepthIndex;
            double closestDist = fabs(currRGBTime - currDepthTime);

            while( currDepth < mDepthFiles.TimeStamps.size() )
            {
                double dtime = mDepthFiles.TimeStamps[currDepth];
                double dt = fabs(currRGBTime - dtime);
                if( IsNear(dtime - currRGBTime) )
                {
                    if( dt < closestDist )
                    {
                        closestDist = dt;
                        closestDepth = currDepth;
                    }
                    currDepth++;
                }
                else
                {
                    break;
                }
            }

            iCurrDepthIndex = closestDepth;
            rgbdList.push_back(std::make_pair(iCurrRGBIndex, iCurrDepthIndex));
            iCurrDepthIndex++;
            iCurrRGBIndex++;
        }
        // drop those rgb and depth images that have no near correspondenc.
        else if( currRGBTime < currDepthTime )
        {
            iCurrRGBIndex++;
        }
        else
        {
            iCurrDepthIndex++;
        }
        loopCount++;
    }

    // now that we have the correspondences, remove unnecessary data.
    FileList rgb, depth;
    for(size_t i = 0; i < rgbdList.size(); i++)
    {
        size_t rgbi = rgbdList[i].first, di = rgbdList[i].second;
        rgb.AddData(mRGBFiles.TimeStamps[rgbi], mRGBFiles.FileNames[rgbi]);
        depth.AddData(mDepthFiles.TimeStamps[di], mDepthFiles.FileNames[di]);
    }

    mRGBFiles = rgb;
    mDepthFiles = depth;
}

void KinectDatasetReader::CorrespondGroundTruths()
{
    // Ground truth data frequency can be higher than the kinect frequency
    size_t iCurrGtIndex = 0, iCurrRGBDIndex = 0;

    //NOTE: This only checks time frame for rgb image, and time stamp of depth image
    //      is not considered.
    double rgbTime, gtTime;

    std::vector<std::pair<size_t, size_t> > detectedPairs;

    int loopCount = 0;
    while( iCurrRGBDIndex < mRGBFiles.TimeStamps.size() &&
            iCurrGtIndex < mGroundTruth.size() )
    {
        gtTime = mGroundTruth[iCurrGtIndex].TimeStamp;
        rgbTime = mRGBFiles.TimeStamps[iCurrRGBDIndex];

        if( IsNear(gtTime - rgbTime) )
        {
            double currGTTime = gtTime;
            double mindt = fabs(currGTTime - rgbTime);
            size_t minIndex = iCurrGtIndex;
            size_t index = iCurrGtIndex;

            while( index < mGroundTruth.size() )
            {
                currGTTime = mGroundTruth[index].TimeStamp;
                double dt = fabs(currGTTime - rgbTime);
                if( IsNear(dt) )
                {
                    if( dt < mindt )
                    {
                        minIndex = index;
                        mindt = dt;
                    }
                    index++;
                }
                else
                    break;
            }

            iCurrGtIndex = minIndex; // set it as minimum distance in time
            detectedPairs.push_back(make_pair(iCurrRGBDIndex, iCurrGtIndex));
            iCurrGtIndex++;
            iCurrRGBDIndex++;
        }
        else if( gtTime < rgbTime )
        {
            iCurrGtIndex++;
        }
        else
        {
            iCurrRGBDIndex++;
        }
        loopCount++;
    }

    // I dont have time to optimize this code, should be done by using list instead of vector. probably!
    GroundTruthArray gtInfoList;
    FileList rgb;
    FileList dp;

    for(size_t i = 0; i < detectedPairs.size(); i++)
    {
        size_t rgbdIndex = detectedPairs[i].first;
        size_t gtIndex = detectedPairs[i].second;
        gtInfoList.push_back(mGroundTruth[gtIndex]);
        rgb.AddData(mRGBFiles.TimeStamps[rgbdIndex], mRGBFiles.FileNames[rgbdIndex]);
        dp.AddData(mDepthFiles.TimeStamps[rgbdIndex], mDepthFiles.FileNames[rgbdIndex]);
    }

    mGroundTruth = gtInfoList;
    mRGBFiles = rgb;
    mDepthFiles = dp;
}

bool KinectDatasetReader::RunSingleCycle()
{
    common::Interface::ScopedLock lock(this);
    
    if( GetCycle() >= (int)mRGBFiles.FileNames.size() )
    {
        mContinueExec = false; // finish execution of kinect.
        return false;
    }
    
    if( mReadFiles )
        LoadNextFiles();
    FinishCycle();
    return true;
}

void KinectDatasetReader::LoadNextFiles()
{
    mTimerLoadTimes->Start();
    
    mRgb = common::KinectImageDiskIO::LoadRgbFromFile(mRGBFiles.FileNames[GetCycle()]);
    mRawDepth = common::KinectImageDiskIO::LoadDepthFromFile(mDepthFiles.FileNames[GetCycle()]);
    
    if( mGenerateFloatDepth )
    {
        mFloatDepth.reset(new common::KinectFloatDepthImage);
        mFloatDepth->Create(mRawDepth->GetWidth(), mRawDepth->GetHeight());
    
        //Generating float depth requires more work.
        const float bad_point = std::numeric_limits<float>::quiet_NaN();
        const common::KinectRawDepthImage::ArrayType& raw_array = mRawDepth->GetArray();
        common::KinectFloatDepthImage::ArrayType& float_array = mFloatDepth->GetArray();
        for(size_t i = 0; i < float_array.size(); i++)
        {
            if( raw_array[i] == 0 )
                float_array[i] = bad_point;
            else
                float_array[i] = raw_array[i] * 0.001f; // Convert to meters
        }
    }
    mTimerLoadTimes->Stop();
    
    if( mGeneratePointCloud )
    {
        mTimerPCGenerator->Start();
        mPC = common::KinectInterface::GeneratePointCloudFromImages(mRgb, mRawDepth, mParams);
        mTimerPCGenerator->Stop();
    }
}

Eigen::Isometry3f KinectDatasetReader::GetCurrentGroundTruth()
{
    return mGroundTruth[GetCycle()].LocalTransform;
}

bool KinectDatasetReader::ProvidesGroundTruth()
{
    return mProvidesGroundTruth;
}


} // end namespace utils
} // end namespace KSRobot
