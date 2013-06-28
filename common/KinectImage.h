#ifndef KSROBOT_COMMON_KINECT_KINECTIMAGE_H
#define KSROBOT_COMMON_KINECT_KINECTIMAGE_H

#include <boost/shared_ptr.hpp>
#include <vector>

namespace KSRobot
{
namespace common
{

template<typename X, int NumChannels, typename Allocator = std::allocator<X> >
class KinectBaseImage
{
public:
    typedef X                                           BaseDataType;
    typedef Allocator                                   AllocatorType;
    typedef KinectBaseImage<X, NumChannels>             ThisType;
    typedef typename boost::shared_ptr<ThisType>        Ptr;
    typedef typename boost::shared_ptr<const ThisType>  ConstPtr;
    typedef typename std::vector<X, Allocator >         ArrayType;
    
    static const int Channels = NumChannels;
public:
    KinectBaseImage(int w, int h) : mWidth(w), mHeight(h) { Create(mWidth, mHeight); }
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

    const X& At(int index) const
    {
        return mData[index];
    }
    
    X& At(int index)
    {
        return mData[index];
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

    int GetNumElements() const
    {
        return mHeight * mWidth * Channels;
    }
    
    size_t ScanLineIndex(int y) const
    {
        return y * mWidth * NumChannels;
    }
    
    static size_t NextIndexUnsafe(size_t idx)
    {
        return idx + NumChannels;
    }
    
private:
    ArrayType                                           mData;
    int                                                 mHeight;
    int                                                 mWidth;
};

typedef KinectBaseImage<unsigned char, 3>               KinectRgbImage;
typedef KinectBaseImage<unsigned short, 1>              KinectRawDepthImage;
typedef KinectBaseImage<float, 1>                       KinectFloatDepthImage;

class KinectImageDiskIO
{
public:
    static void                         SaveToFileRgb(const std::string& file, KinectRgbImage::ConstPtr rgb);
    static void                         SaveToFileDepth(const std::string& file, KinectRawDepthImage::ConstPtr depth);
    
    static KinectRgbImage::Ptr          LoadRgbFromFile(const std::string& file);
    static KinectRawDepthImage::Ptr     LoadDepthFromFile(const std::string& file);
};

} // end of namespace utils
} // end of namespace KSRobot

#endif //KSROBOT_COMMON_KINECT_KINECTIMAGE_H
