#include <interfaces/KinectDatasetReader.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/static_assert.hpp>
#include <cmath>

#include <pcl/point_cloud.h> //basic pcl includes
#include <pcl/point_types.h>
#include <pcl/io/image_grabber.h>

namespace KSRobot
{
namespace interfaces
{

using boost::lexical_cast;
using namespace std;
using namespace Eigen;
namespace fs = boost::filesystem;
namespace gil = boost::gil;

KinectDatasetReader::KinectDatasetReader(common::ProgramOptions::Ptr po, const std::string& name) : KinectInterface(po, name), 
    mCycle(0), mRunning(false)
{
}

KinectDatasetReader::~KinectDatasetReader()
{
}

//TODO: Change this as a configuration constant
#define KINECT_TIME_DIFFERENCE (0.5 / 30.0)
static inline bool IsNear(double diff)
{
    return std::fabs(diff) <= KINECT_TIME_DIFFERENCE;
}

void KinectDatasetReader::Initialize(const std::string& pathStr)
{
    fs::path dirPath(pathStr);
    fs::path rgbIndexFile = dirPath / "rgb.txt";
    fs::path depthIndexFile = dirPath / "depth.txt";
    fs::path gtPath = dirPath / "groundtruth.txt";

    ifstream depthFile(depthIndexFile.string().c_str(), ios::in);
    ifstream rgbFile(rgbIndexFile.string().c_str(), ios::in);

    ReadIndexFile(rgbFile, dirPath, mRGBFiles.TimeStamps, mRGBFiles.FileNames);
    rgbFile.close();
    ReadIndexFile(depthFile, dirPath, mDepthFiles.TimeStamps, mDepthFiles.FileNames);
    depthFile.close();

    // Now read the ground truth data.
    ifstream gtFile(gtPath.string().c_str(), ios::in);
    ReadGroundTruthData(gtFile, mGroundTruth);
    gtFile.close();
    // Now check which indexes correspond together
    CorrespondRGBDIndices();
    // Now check those indexes with ground truth data.
    CorrespondGroundTruth();
    //TODO: Add some code to move the ground truth to (0,0,0), (0,0,0)

    // Now create point cloud generator
    mPointCloudGenerator.reset(new ImageGrabber(mDepthFiles.FileNames, 0, false));
    mPointCloudGenerator->setDepthImageUnits(5000.0f);
    mPointCloudGenerator->setRGBImageFiles(mRGBFiles.FileNames);
    boost::function<void ( common::KinectPointCloud::Ptr)> f =
        boost::bind(&KinectDatasetReader::PointCloudCallback, this, _1);
    mPointCloudGenerator->registerCallback(f);
}

void KinectDatasetReader::PointCloudCallback(common::KinectPointCloud::Ptr pc)
{
    //TODO: For openni grabber it is safe, but I don't know for imagegrabber
    mPC = pc;
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
        if( strs.size() != 2 )
        {
            throw std::runtime_error("Invalid dataset file");
            continue;
        }

        timeStamps.push_back(boost::lexical_cast<double>(strs[0]));
        filelist.push_back((fullPath / strs[1]).string());
    }
}

void KinectDatasetReader::ReadGroundTruthData(ifstream& file,
        vector< KinectDatasetReader::GroundTruthInfo >& gtInfo)
{
    gtInfo.clear();
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

        gt.TimeStamp = lexical_cast<double>(strs[0]);
        for(int i = 0; i < 3; i++)
            gt.Position[i] = lexical_cast<float>(strs[i + 1]);

        // in form of wxyz
        gt.Rotation = Quaternionf(lexical_cast<float>(strs[7]),
                                  lexical_cast<float>(strs[4]),
                                  lexical_cast<float>(strs[5]),
                                  lexical_cast<float>(strs[6]));

        gtInfo.push_back(gt);
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

void KinectDatasetReader::CorrespondGroundTruth()
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
            //TODO: In future, maybe add interpolation for gt data if not available for curr cycle?
            iCurrRGBDIndex++;
        }
        loopCount++;
    }

    // I dont have time to optimize this code, should be done by using list instead of vector. probably!
    std::vector<GroundTruthInfo> gtInfoList;
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
    if( mCycle >= mRGBFiles.FileNames.size() )
    {
        //TODO: This means dataset is finished. how can we send finished signal?
        return false;
    }
    
    LockData();
    
    LoadNextFiles();
    //This will trigger the callbacks registered to the pointcloud.
    mPointCloudGenerator->start();
    mCycle++;
    
    UnlockData();
    return true;
}

static void CopyToKinectImage(common::KinectRgbImage::Ptr dst, gil::rgb8_image_t& src)
{
    common::KinectRgbImage::ArrayType& arr = dst->GetArray();
    gil::rgb8_view_t view = gil::view(src);
    size_t index = 0;
    for(int i = 0; i < src.height(); i++)
    {
        gil::rgb8_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < src.width(); j++)
        {
            gil::rgb8_pixel_t& pix = iter[j];
            for(int k = 0; k < common::KinectRgbImage::Channels; k++)
                arr[index++] = pix[k];
        }
    }
}

static void CopyToKinectImage(common::KinectFloatDepthImage::Ptr dst, gil::gray16_image_t& src)
{
    common::KinectFloatDepthImage::ArrayType& arr = dst->GetArray();
    gil::gray16_view_t view = gil::view(src);
    size_t index = 0;
    for(int i = 0; i < src.height(); i++)
    {
        gil::gray16_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < src.width(); j++)
        {
            arr[index++] = static_cast<float>(iter[j]) / 5.0f;
        }
    }
}

static void CopyToKinectImage(common::KinectRawDepthImage::Ptr dst, gil::gray16_image_t& src)
{
    common::KinectRawDepthImage::ArrayType& arr = dst->GetArray();
    gil::gray16_view_t view = gil::view(src);
    size_t index = 0;
    for(int i = 0; i < src.height(); i++)
    {
        gil::gray16_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < src.width(); j++)
        {
            arr[index++] = iter[j] / 5;
        }
    }
}

void KinectDatasetReader::LoadNextFiles()
{
    gil::rgb8_image_t rgb;
    gil::gray16_image_t depth;
    gil::png_read_image(mRGBFiles.FileNames[mCycle], rgb);
    gil::png_read_image(mDepthFiles.FileNames[mCycle], depth);

    mRgb->Create(rgb.width(), rgb.height());
    mRawDepth->Create(depth.width(), depth.height());
    mFloatDepth->Create(depth.width(), depth.height());

    CopyToKinectImage(mRgb, rgb);
    CopyToKinectImage(mRawDepth, depth);
    CopyToKinectImage(mFloatDepth, depth);
}

} // end namespace utils
} // end namespace KSRobot
