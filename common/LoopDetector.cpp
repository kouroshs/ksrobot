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
#include <boost/random/uniform_int_distribution.hpp>
#include <ctime>
#include <math.h>
#include <Eigen/Dense>

#define DEFAULT_NUM_CANDIDATE_IMAGES            10
#define DEFAULT_USE_FOVIS_DESCRIPTOR            true
#define DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR      false
#define DEFAULT_FEATURE_DETECTOR_NAME           std::string("SIFT")
#define DEFAULT_DESCRIPTOR_EXTRACTOR_NAME       std::string("SIFT")
#define DEFAULT_MINIMUM_FEATURE_MATCHES         50
#define DEFAULT_RANSAC_MIN_SET_SIZE             3
#define DEFAULT_RANSAC_MAX_STEPS                100
#define DEFAULT_RANSAC_GOOD_MODEL_PERCENTAGE    0.7f
#define DEFAULT_RANSAC_MIN_INLIERS              10
#define DEFAULT_RANSAC_MAX_ACCEPTABLE_ERROR     0.1 //TODO
#define DEFAULT_RANSAC_INLIER_MAX_SQUARED_DIST  0.05 //TODO

#ifndef USE_PREDICTABLE_RANDOM_BEHAVIOUR
#define DEFAULT_RANSAC_RNG_INITIALIZER          std::time(0) 
#else
#define DEFAULT_RANSAC_RNG_INITIALIZER          
#endif //USE_PREDICTABLE_RANDOM_BEHAVIOUR

#define FOVIS_KEYPOINT_SIZE            80   //TODO: Move this definition to FovisInterface

static inline size_t diff(size_t a, size_t b)
{
    return a > b ? a - b : b - a;
}

static inline float GetAngle(const Eigen::Isometry3f& iso)
{
    Eigen::Matrix<float,3,1> euler = iso.rotation().eulerAngles(2, 1, 0);
    float pitch = euler(1, 0) * 180.0 / M_PI;
    return pitch;
}

namespace KSRobot
{
namespace common
{

LoopDetector::LoopDetector(): Interface(), mNumCandidateImages(DEFAULT_NUM_CANDIDATE_IMAGES), mUseFovisDescriptor(DEFAULT_USE_FOVIS_DESCRIPTOR),
    mCurrSceneDescriptor(NULL), mNormalizeFovisDescriptor(DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR),
    mFeatureDetectorName(DEFAULT_FEATURE_DETECTOR_NAME), mDescriptorExtractorName(DEFAULT_DESCRIPTOR_EXTRACTOR_NAME),
    mMinimumFeatureMatches(DEFAULT_MINIMUM_FEATURE_MATCHES),
    mRansacMinSetSize(DEFAULT_RANSAC_MIN_SET_SIZE), mRansacMaxSteps(DEFAULT_RANSAC_MAX_STEPS),
    mRansacGoodModelPercentage(DEFAULT_RANSAC_GOOD_MODEL_PERCENTAGE), mRansacMinInliers(DEFAULT_RANSAC_MIN_INLIERS),
    mRansacMaxAcceptableError(DEFAULT_RANSAC_MAX_ACCEPTABLE_ERROR), mRansacInlierMaxSquaredDist(DEFAULT_RANSAC_INLIER_MAX_SQUARED_DIST),
    mDelayLoopDetections(0), mLoopCandidatesMaxRange(10000.0f), mLoopCandidatesMaxYawDifference(1000000.0f)
{
    mRansacRNG.seed(DEFAULT_RANSAC_RNG_INITIALIZER);
}

LoopDetector::~LoopDetector()
{
    if( mCurrSceneDescriptor )
        delete mCurrSceneDescriptor;
}

void LoopDetector::ReadSettings(ProgramOptions::Ptr po)
{
    common::Interface::ReadSettings(po);
    mRansacMinSetSize           = po->GetInt    ("RANSAC.MinSetSize",           DEFAULT_RANSAC_MIN_SET_SIZE         );
    mRansacMaxSteps             = po->GetInt    ("RANSAC.MaxSteps",             DEFAULT_RANSAC_MAX_STEPS            );
    mRansacGoodModelPercentage  = po->GetDouble ("RANSAC.GoodModelPercentage",  DEFAULT_RANSAC_GOOD_MODEL_PERCENTAGE);
    mRansacMinInliers           = po->GetInt    ("RANSAC.MinInliers",           DEFAULT_RANSAC_MIN_INLIERS          );
    mRansacMaxAcceptableError   = po->GetDouble ("RANSAC.MaxAcceptableError",   DEFAULT_RANSAC_MAX_ACCEPTABLE_ERROR );
    mRansacInlierMaxSquaredDist = po->GetDouble ("RANSAC.InlierMaxSquaredDist", DEFAULT_RANSAC_INLIER_MAX_SQUARED_DIST);
    mNumCandidateImages         = po->GetInt    ("NumCandidateImages",          DEFAULT_NUM_CANDIDATE_IMAGES        );
    mUseFovisDescriptor         = po->GetBool   ("UseFovisDescriptor",          DEFAULT_USE_FOVIS_DESCRIPTOR        );
    mNormalizeFovisDescriptor   = po->GetBool   ("NormalizeFovisDescriptor",    DEFAULT_NORMALIZE_FOVIS_DESCRIPTOR  );
    mFeatureDetectorName        = po->GetString ("FeatureDetector",             DEFAULT_FEATURE_DETECTOR_NAME       );
    mDescriptorExtractorName    = po->GetString ("DescriptorExtractor",         DEFAULT_DESCRIPTOR_EXTRACTOR_NAME   );
    mMinimumFeatureMatches      = po->GetInt    ("MinimumFeatureMatches",       DEFAULT_MINIMUM_FEATURE_MATCHES     );
    mDelayLoopDetections        = po->GetInt    ("DelayLoopDetections",         0                                   );
    mLoopCandidatesMaxYawDifference = po->GetDouble("LoopCandidatesMaxYawDifference", 1000000.0                     ) * M_PI / 180.0;
    mLoopCandidatesMaxRange     = po->GetDouble ("LoopCandidatesMaxRange",      1000000.0                           ) * M_PI / 180.0;
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
    if( !mUseFovisDescriptor ) // If we are not using fovis, then we need depth to extract 3d positions.
    {
        kd.RawDepth = mVO->GetCurrentRawDepthImage();
        kd.Image = mVO->GetCurrentRgbImage();
    }
    mKeyframesQueue.push(kd);
}

bool LoopDetector::RunSingleCycle()
{
    KeyframeData kd;
    std::vector<int> candidates;
    Eigen::Isometry3f transform;
    int count = 0;
    std::vector<double> out_scores;
    while( mKeyframesQueue.try_pop(kd) )
    {
        InternalKeyframeInfo internal_info;
        internal_info.Cycle = kd.Keyframe->CurrentCycle;
        internal_info.Pose = kd.Keyframe->GlobalPose;
        mInternalKeyframesBookkeeping.push_back(internal_info);
        
        count++;
        ExtractImageKeypointDescriptors(kd);
        ExtractSceneDescriptors();
        
        if( GetCycle() > 0 )
        {
            candidates.clear();
            //For now, I dont care about score.
            incremental_vtree::VocabularyTreeRetriever::Retrieve(mVTree, *mCurrSceneDescriptor, mNumCandidateImages, &candidates, &out_scores);
            //For each candidate, check for loop closure. For the first one found terminate search
            for(size_t i = 0; i < candidates.size(); i++)
                if( CheckForLoopClosure(candidates[i], transform) )
                {
                    LoopClosure lc;
                    lc.Transform = transform;
                    lc.Cycle1 = candidates[i];
                    lc.Cycle2 = mVTree.GetNumImages() + 1; // TODO: What is this cycle???
                    mOnLoopDetected(lc);
                    break;
                }
        }
        mVTree.InsertImage(*mCurrSceneDescriptor);
        FinishCycle();
    }
    return count != 0;
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
        for(size_t i = 0; i < pointList.size(); i++)
            pointList[i] = kd.Keyframe->Features[i].RelativePosition;
    }
    else
    {
        common::KinectInterface::Ptr kinect = mVO->GetKinect();
        for(size_t i = 0; i < pointList.size(); i++)
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
    //NOTE: First impose some restrictions on what candidates can be checked for the actual loop closure test.
    if( (int)diff(mInternalKeyframesBookkeeping.back().Cycle, candidate) < mDelayLoopDetections )
        return false; // if it is too soon for loop closure test
    if( (mInternalKeyframesBookkeeping.back().Pose.translation() - 
        mInternalKeyframesBookkeeping[candidate].Pose.translation()).squaredNorm() > mLoopCandidatesMaxRange )
        return false; // if two candidates are very far to one another
    //now calculate yaw (in kinect framework pitch angle)
    if( fabs(GetAngle(mInternalKeyframesBookkeeping.back().Pose) - GetAngle(mInternalKeyframesBookkeeping[candidate].Pose)) >
        mLoopCandidatesMaxYawDifference )
        return false; // if the angle difference makes no sense to test for loop closure.
    
    std::vector<cv::DMatch> all_matches, good_matches;

    MatchFeatures(candidate, all_matches);
    ExtractValidMatches(all_matches, mPointArrayList[candidate], mPointArrayList.back(), good_matches);
    
    if( good_matches.size() < mMinimumFeatureMatches )
        return false;
    
    if( !Ransac(mPointArrayList[candidate], mPointArrayList.back(), good_matches, transform) )
        return false;
    
    //TODO: REFINE Ransac result
    
    return true;
}

static inline bool IsValidPoint(const Eigen::Vector3f& p)
{
    return p[0] != std::numeric_limits<float>::quiet_NaN();
}

void LoopDetector::ExtractValidMatches(const std::vector<cv::DMatch>& matches, const PointArray& prev, const PointArray&curr, 
                                       std::vector<cv::DMatch>& out)
{
    for(size_t i = 0; i < matches.size(); i ++)
    {
        const cv::DMatch& match = matches[i];
        if( IsValidPoint(curr[match.queryIdx]) && IsValidPoint(prev[match.trainIdx]) )
            out.push_back(match);
    }
}

void LoopDetector::MatchFeatures(int candidate, std::vector<cv::DMatch>& matches)
{
    std::vector<cv::DMatch> allMatches;
    mMatcher.match(CurrentDescriptor(), mKeyframeImageKeypointDescriptors[candidate], allMatches);
    
    // now keep only good matches
    float min_dist = std::numeric_limits<float>::max();
    
    for(std::vector<cv::DMatch>::iterator iter = allMatches.begin(); iter != allMatches.end(); iter++)
        if( iter->distance < min_dist )
            min_dist = iter->distance;
    
    for(std::vector<cv::DMatch>::iterator iter = allMatches.begin(); iter != allMatches.end(); iter++)
        if( iter->distance <= 2 * min_dist )
            matches.push_back(*iter);
}

static inline void PermuteVector(std::vector<int>& permute, boost::random::mt19937& gen)
{
    for(int i = permute.size() - 1; i > 0; i--)
    {
        boost::random::uniform_int_distribution<int> rng(0, i);
        std::swap(permute[i], permute[rng(gen)]);
    }
}

bool LoopDetector::Ransac(const PointArray& prev_points, const PointArray& curr_points, 
                          const std::vector<cv::DMatch>& matches, Eigen::Isometry3f& transform)
{
    const size_t min_inliers = matches.size() * mRansacGoodModelPercentage;
    float best_error = std::numeric_limits<float>::max();
    Eigen::Isometry3f best_model, trans;
    size_t best_inliers_count = 0;
    
    std::vector<int> permute(matches.size());
    for(size_t i = 0; i < permute.size(); i++)
        permute[i] = i;
    
    std::vector<int> consensus_set;
    
    for(size_t ransac_step = 0; ransac_step < mRansacMaxSteps; ransac_step++)
    {
        PermuteVector(permute, mRansacRNG);
        //choose n first as maybe inliers
        consensus_set.clear();
        consensus_set.assign(permute.begin(), permute.begin() + mRansacMinSetSize);
        
        if( ComputeHornTransform(prev_points, curr_points, matches, consensus_set, trans) )
            continue;
        // now compute how many points are inliers for this model.
        consensus_set.clear();
        for(size_t i = 0; i < permute.size(); i++)
            if( IsInlier(prev_points[i], curr_points[i], trans) )
                consensus_set.push_back(i);
        
        if( consensus_set.size() > min_inliers )
        {
            // This is a good set, first update the model using all inliers
            // then estimate error on the inliers
            if( ComputeHornTransform(prev_points, curr_points, matches, consensus_set, trans) )
                continue; //NOTE: Should never happen
            float error = ComputeError(prev_points, curr_points, matches, consensus_set, trans);
            if( error < best_error )
            {
                best_error = error;
                best_model = trans;
                best_inliers_count = consensus_set.size();
            }
        }
    }
    
    transform = best_model;
    
    if( best_inliers_count < mRansacMinInliers && best_error <= mRansacMaxAcceptableError )
        return false;
    return true;
}

bool LoopDetector::ComputeHornTransform(const PointArray& prev_points, const PointArray& curr_points,
                                        const std::vector<cv::DMatch>& matches, 
                                        const std::vector<int>& indexer,
                                        Eigen::Isometry3f& transform)
{
    Eigen::Vector3f center_prev(0, 0, 0), center_curr(0, 0, 0);
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    Eigen::Matrix4f N;
    for(size_t i = 0; i < indexer.size(); i++)
    {
        center_curr += curr_points[matches[indexer[i]].queryIdx];
        center_prev += prev_points[matches[indexer[i]].trainIdx];
    }
    center_curr /= indexer.size();
    center_prev /= indexer.size();
    
    for(size_t i = 0; i < indexer.size(); i++)
    {
        Eigen::Vector3f curr = curr_points[matches[indexer[i]].queryIdx] - center_curr;
        Eigen::Vector3f prev = prev_points[matches[indexer[i]].trainIdx] - center_prev;
        //TODO: Should it be in this order or reverse order?
        M += curr * prev.transpose();
    }
    
    float Sxx = M(0, 0);
    float Sxy = M(0, 1);
    float Sxz = M(0, 2);
    float Syx = M(1, 0);
    float Syy = M(1, 1);
    float Syz = M(1, 2);
    float Szx = M(2, 0);
    float Szy = M(2, 1);
    float Szz = M(2, 2);
    
    N <<    Sxx + Syy + Szz , Syz - Szy         , Szx - Sxz         , Sxy - Syx     ,
            Syz - Szy       , Sxx - Syy - Szz   , Sxy + Syx         , Szx + Sxz     ,
            Szx - Sxz       , Sxy + Syx         , Syy - Sxx - Szz   , Syz + Szy     ,
            Sxy - Syx       , Szx + Sxz         , Syz + Szy         , Szz - Sxx - Syy;
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(N);
    if( solver.info() != Eigen::Success )
        return false;
    
    Eigen::Vector4f largestEigval = solver.eigenvectors().col(3);
    Eigen::Quaternionf quat(largestEigval);
    
    transform.linear() = quat.toRotationMatrix();
    transform.translation().setZero();
    
    for(size_t i = 0; i < indexer.size(); i++)
        transform.translation() += curr_points[matches[indexer[i]].queryIdx] - transform.linear() * prev_points[matches[indexer[i]].trainIdx];
    transform.translation() /= indexer.size();
    
    return true;
}

bool LoopDetector::IsInlier(const Eigen::Vector3f& prev, const Eigen::Vector3f& curr, const Eigen::Isometry3f& transform)
{
    return (curr - transform * prev).squaredNorm() <= mRansacInlierMaxSquaredDist;
}

float LoopDetector::ComputeError(const PointArray& prev_points, const PointArray& curr_points, const std::vector<cv::DMatch>& matches,
                                 const std::vector<int>& consensus_set, const Eigen::Isometry3f& trans)
{
    float total_error = 0.0f;
    for(size_t i = i; i <= consensus_set.size(); i++)
        total_error += (curr_points[matches[consensus_set[i]].queryIdx] - trans * prev_points[matches[consensus_set[i]].trainIdx]).squaredNorm();
    return sqrtf(total_error) / consensus_set.size();
}

};
};
