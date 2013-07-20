/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <common/LoopDetector.h>
#include <trees/vocabulary-tree-retriever.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEFAULT_NUM_CANDIDATE_IMAGES            10
#define DEFAULT_USE_FOVIS_DESCRIPTOR            true
#define DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR      false
#define DEFAULT_FEATURE_DETECTOR_NAME           std::string("SIFT")
#define DEFAULT_DESCRIPTOR_EXTRACTOR_NAME       std::string("SIFT")
#define DEFAULT_MINIMUM_FEATURE_MATCHES         50

#define FOVIS_KEYPOINT_SIZE            80   //TODO: Move this definition to FovisInterface

namespace KSRobot
{
namespace common
{

LoopDetector::LoopDetector(): Interface(), mNumCandidateImages(DEFAULT_NUM_CANDIDATE_IMAGES), mUseFovisDescriptor(DEFAULT_USE_FOVIS_DESCRIPTOR),
    mCurrSceneDescriptor(NULL), mNormalizeFovisDescriptor(DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR),
    mFeatureDetectorName(DEFAULT_FEATURE_DETECTOR_NAME), mDescriptorExtractorName(DEFAULT_DESCRIPTOR_EXTRACTOR_NAME),
    mMinimumFeatureMatches(DEFAULT_MINIMUM_FEATURE_MATCHES)
{
}

LoopDetector::~LoopDetector()
{
    if( mCurrSceneDescriptor )
        delete mCurrSceneDescriptor;
}

void LoopDetector::ReadSettings(ProgramOptions::Ptr po)
{
    mNumCandidateImages         = po->GetInt    ("NumCandidateImages",          DEFAULT_NUM_CANDIDATE_IMAGES);
    mUseFovisDescriptor         = po->GetBool   ("UseFovisDescriptor",          DEFAULT_USE_FOVIS_DESCRIPTOR);
    mNormalizeFovisDescriptor   = po->GetBool   ("NormalizeFovisDescriptor",    DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR);
    mFeatureDetectorName        = po->GetString ("FeatureDetector",             DEFAULT_FEATURE_DETECTOR_NAME);
    mDescriptorExtractorName    = po->GetString ("DescriptorExtractor",         DEFAULT_DESCRIPTOR_EXTRACTOR_NAME);
    mMinimumFeatureMatches      = po->GetInt    ("MinimumFeatureMatches",       DEFAULT_MINIMUM_FEATURE_MATCHES);
}

void LoopDetector::RegisterToVO(VisualOdometryInterface::Ptr vo)
{
    mVO = vo;
    mVO->RegisterKeyframeReceiver(boost::bind(&LoopDetector::OnNewKeyframe, this));
}

boost::signals2::connection LoopDetector::RegisterLoopReceiver(boost::function<void(const LoopClosure& lc)> fn)
{
    return mOnLoopDetected.connect(fn);
}

void LoopDetector::OnNewKeyframe()
{
    KeyframeData kd;
    kd.Keyframe = mVO->GetLatestKeyframe();
    kd.Image = mVO->GetCurrentRgbImage();
    if( !mUseFovisDescriptor ) // If we are not using fovis, then we need depth to extract 3d positions.
        kd.RawDepth = mVO->GetCurrentRawDepthImage();
    mKeyframesQueue.push(kd);
}

bool LoopDetector::RunSingleCycle()
{
    KeyframeData kd;
    std::vector<int> candidates;
    Eigen::Isometry3f transform;
    while( mKeyframesQueue.try_pop(kd) )
    {
        ExtractImageKeypointDescriptors(kd);
        ExtractSceneDescriptors();
        candidates.clear();
        //For now, I dont care about score.
        incremental_vtree::VocabularyTreeRetriever::Retrieve(mVTree, *mCurrSceneDescriptor, mNumCandidateImages, &candidates, NULL);
        //For each candidate, check for loop closure. For the first one found terminate search
        bool bLoopClosure;
        size_t loop_index;
        for(size_t i = 0; i < candidates.size(); i++)
            if( (bLoopClosure = CheckForLoopClosure(candidates[i], transform)) )
            {
                loop_index = i;
                break;
            }
            
        if( bLoopClosure )
        {
            //Notify every listener
            LoopClosure lc;
            lc.Transform = transform;
            lc.Cycle1 = candidates[loop_index];
            lc.Cycle2 = mVTree.GetNumImages() + 1; // TODO: What is this cycle???
            
            mOnLoopDetected(lc);
        }
        mVTree.InsertImage(*mCurrSceneDescriptor);
    }
}

void LoopDetector::ExtractImageKeypointDescriptors(const LoopDetector::KeyframeData& kd)
{
    cv::Mat desc;
    std::vector<cv::KeyPoint> keypoints; // in case of using fovis, this will be empty
    if( mUseFovisDescriptor )
    {        
        const float mlt = mNormalizeFovisDescriptor ? 1.0f / 255.0f : 1.0f;
        const int stride = kd.Keyframe->DataPool.FeatureStride;
        
        desc = cv::Mat(kd.Keyframe->Features.size(), kd.Keyframe->DataPool.FeatureLength, CV_32F);
        
        for(int i = 0; i < desc.rows; i++)
        {
            float* matPtr = desc.ptr<float>(i);
            int offset = i * stride;
            for(int j = 0; j < (int)kd.Keyframe->DataPool.FeatureLength; j++)
                matPtr[j] = (float)kd.Keyframe->DataPool[offset + j] * mlt;
        }
    }
    else
    {
        if( mDescExtractor.empty() || mFeatureDetector.empty() )
        {
            mDescExtractor = cv::DescriptorExtractor::create(mDescriptorExtractorName);
            mFeatureDetector = cv::FeatureDetector::create(mFeatureDetectorName);
            
            if( mDescExtractor.empty() || mFeatureDetector.empty() )
            {
                mFeatureDetector = cv::FeatureDetector::create("SIFT");
                mDescExtractor = cv::DescriptorExtractor::create("SIFT");
            }
        }
        
        cv::Mat colorImage(kd.Image->GetHeight(), kd.Image->GetWidth(), CV_8UC3, (void*)kd.Image->GetArray().data());
        cv::Mat image;
        cv::cvtColor(colorImage, image, CV_BGR2RGB); // convert rgb to bgr
        cv::cvtColor(image, image, CV_BGR2GRAY); // convert bgr to gray
        
        mFeatureDetector->detect(image, keypoints);
        mDescExtractor->compute(image, keypoints, desc);
    }
    
    mKeyframeImageKeypointDescriptors.push_back(desc);
    
    //Now extract 3d positions
    Extract3DPositions(kd, keypoints);
}

void LoopDetector::Extract3DPositions(const KeyframeData& kd, const std::vector<cv::KeyPoint>& keypoints)
{
    mPointArrayList.push_back(std::vector<Eigen::Vector3f>());
    std::vector<Eigen::Vector3f>& pointList = mPointArrayList.back();
    pointList.resize(CurrentDescriptor().rows); // faster than reserve and push_back
    
    if( mUseFovisDescriptor )
    {
        // no need to calculate 3d positions
        for(int i = 0; i < pointList.size(); i++)
            pointList[i] = kd.Keyframe->Features[i].RelativePosition;
    }
    else
    {
        common::KinectInterface::Ptr kinect = mVO->GetKinect();
        for(int i = 0; i < pointList.size(); i++)
        {
            cv::Point2f pt = keypoints[i].pt;
            kinect->Get3DPosition(pt.x, pt.y, kd.RawDepth->At(kd.RawDepth->GetElementStartIndex(pt.x, pt.y)), pointList[i]);
        }
    }
}

void LoopDetector::ExtractSceneDescriptors()
{
    int dim = CurrentDescriptor().cols;
    int num_points = CurrentDescriptor().rows;
    
    if( mCurrSceneDescriptor == NULL )
        mCurrSceneDescriptor = new PointSet(dim);

    mCurrSceneDescriptor->clear();
    
    for(int i = 0; i < num_points; i++)
    {
        Point p(dim);
        const float* dataPtr = CurrentDescriptor().ptr<float>(i);
        for(int j = 0; j < dim; j++)
            p[j] = dataPtr[j];
        mCurrSceneDescriptor->add_point(p);
    }
}

bool LoopDetector::CheckForLoopClosure(int candidate, Eigen::Isometry3f& transform)
{
    std::vector<cv::DMatch> matches;
    std::vector<int> idx_prevcycle, idx_currcycle;

    MatchFeatures(candidate, matches);    
    ExtractValidMatches(matches, candidate, mPointArrayList.size(), idx_prevcycle, idx_currcycle);
    
    if( idx_currcycle.size() < mMinimumFeatureMatches )
        return false;
    
    if( !Ransac(candidate, mPointArrayList.size(), idx_prevcycle, idx_currcycle, transform) )
        return false;
    
    //TODO: REFINE Ransac result
    
    return true;
}

static inline bool IsValidPoint(const Eigen::Vector3f& p)
{
    return p[0] != std::numeric_limits<float>::quiet_NaN();
}

void LoopDetector::ExtractValidMatches(const std::vector<cv::DMatch>& matches, int prevcycle, int currcycle, 
                                       std::vector<int>& idx_prevcycle, std::vector<int>& idx_currcycle)
{
    std::vector<Eigen::Vector3f> &prev_points = mPointArrayList[prevcycle];
    std::vector<Eigen::Vector3f> &curr_points = mPointArrayList[currcycle];
    for(int i = 0; i < matches.size(); i ++)
    {
        const cv::DMatch& match = matches[i];
        
        if( IsValidPoint(curr_points[match.queryIdx]) && IsValidPoint(prev_points[match.trainIdx]) )
        {
            idx_currcycle.push_back(match.queryIdx);
            idx_prevcycle.push_back(match.trainIdx);
        }
    }
}

void LoopDetector::MatchFeatures(int candidate, std::vector<cv::DMatch>& matches)
{
    std::vector<cv::DMatch> allMatches;
    mMatcher.match(CurrentDescriptor(), mKeyframeImageKeypointDescriptors[candidate], allMatches);
    
    // now keep only good matches
    float min_dist = 10000.0f;
    
    for(std::vector<cv::DMatch>::iterator iter = allMatches.begin(); iter != allMatches.end(); iter++)
        if( iter->distance < min_dist )
            min_dist = iter->distance;
    
    for(std::vector<cv::DMatch>::iterator iter = allMatches.begin(); iter != allMatches.end(); iter++)
        if( iter->distance <= 2 * min_dist )
            matches.push_back(*iter);
}

bool LoopDetector::Ransac(int prev_cycle, int curr_cycle, const std::vector<int>& prev_idx, const std::vector<int>& curr_idx, 
                          Eigen::Isometry3f& transform)
{
    return false;
}

};
};
