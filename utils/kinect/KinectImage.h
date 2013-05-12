#ifndef KSROBOT_UTILS_KINECT_KINECTIMAGE_H
#define KSROBOT_UTILS_KINECT_KINECTIMAGE_H

#include <boost/shared_ptr.hpp>
#include <vector>

namespace KSRobot
{
namespace utils
{
//NOTE: This class is provided to remove OpenCV dependency.
template<class X, int NumChannels>
class KinectBaseImage
{
private:
    typedef KinectBaseImage<X, NumChannels>                ThisType;
public:
    typedef typename boost::shared_ptr<ThisType>        Ptr;
    typedef typename boost::shared_ptr<const ThisType>  ConstPtr;
    typedef typename std::vector<X>                     ArrayType;
    
    static const int Channels = NumChannels;
public:
    KinectBaseImage(int w, int h) : mWidth(0), mHeight(0) { Create(mWidth, mHeight); }
    KinectBaseImage() : mWidth(0), mHeight(0) {;}
    KinectBaseImage(const KinectBaseImage<X, NumChannels>& o) : mWidth(o.GetWidth()), mHeight(o.GetHeight()),
                                                                mData(GetArray()) {;}
    ~KinectBaseImage(){;}

    void Create(int w, int h)
    {
        if( mWidth * mHeight != w * h /*mWidth != w || mHeight != h*/ )
        {
            //recreate the array
            mWidth = w;
            mHeight = h;
            //mData.reset(new X[w * h * NumChannels]);
            mData.resize(w * h * NumChannels);
        }
    }

    const ArrayType& GetArray() const
    {
        return mData;
    }
    
    ArrayType& GetArray()
    {
        return mData;
    }

    int GetWidth() const
    {
        return mWidth;
    }
    
    int GetStride() const
    {
        return mWidth * sizeof(X) * NumChannels;
    }
    
    int GetHeight() const
    {
        return mHeight;
    }

private:
    ArrayType                                           mData;
    int                                                 mHeight;
    int                                                 mWidth;
};

typedef KinectBaseImage<unsigned char, 3>               KinectRgbImage;
typedef KinectBaseImage<unsigned short, 1>              KinectRawDepthImage;
typedef KinectBaseImage<float, 1>                       KinectFloatDepthImage;
} // end of namespace utils
} // end of namespace KSRobot

#endif //KSROBOT_UTILS_KINECT_KINECTIMAGE_H
