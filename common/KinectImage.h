#ifndef KSROBOT_COMMON_KINECT_KINECTIMAGE_H
#define KSROBOT_COMMON_KINECT_KINECTIMAGE_H

#include <boost/shared_ptr.hpp>
#include <common/Defenitions.h>
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
    KinectBaseImage(int w, int h) : mWidth(0), mHeight(0) { Create(w, h); }
    KinectBaseImage() : mWidth(0), mHeight(0) {;}
    KinectBaseImage(const KinectBaseImage<X, NumChannels>& o) : mWidth(o.GetWidth()), mHeight(o.GetHeight()),
                                                                mData(GetArray()) {;}
    ~KinectBaseImage(){;}

    inline void Create(int w, int h)
    {
        if( mWidth != w || mHeight != h )
        {
            //recreate the array
            mWidth = w;
            mHeight = h;
            mData.resize(w * h * NumChannels);
        }
    }

    inline const ArrayType& GetArray() const
    {
        return mData;
    }
    
    inline ArrayType& GetArray()
    {
        return mData;
    }

    inline const X& At(int index) const
    {
        return mData[index];
    }
    
    inline X& At(int index)
    {
        return mData[index];
    }
    
    inline int GetWidth() const
    {
        return mWidth;
    }
    
    inline int GetStride() const
    {
        return mWidth * sizeof(X) * NumChannels;
    }
    
    inline int GetHeight() const
    {
        return mHeight;
    }

    inline int GetNumElements() const
    {
        return mHeight * mWidth * Channels;
    }
    
    inline size_t ScanLineIndex(size_t y) const
    {
        return y * mWidth * NumChannels;
    }
    
    inline static size_t NextIndexUnsafe(size_t idx)
    {
        return idx + NumChannels;
    }
    
    inline size_t GetElementStartIndex(size_t x, size_t y) const
    {
        return ScanLineIndex(y) + x * NumChannels;
    }
    
private:
    ArrayType                                           mData;
    int                                                 mHeight;
    int                                                 mWidth;
};

typedef KinectBaseImage<unsigned char, 3>               KinectRgbImage;
typedef KinectBaseImage<unsigned char, 1>               KinectGrayImage;
typedef KinectBaseImage<unsigned short, 1>              KinectRawDepthImage;
typedef KinectBaseImage<float, 1>                       KinectFloatDepthImage;

class KinectImageDiskIO
{
public:
    static void                         SaveToFileRgb(const std::string& file, KinectRgbImage::ConstPtr rgb);
    static void                         SaveToFileDepth(const std::string& file, KinectRawDepthImage::ConstPtr depth);
    static void                         SaveToFileGray(const std::string& file, KinectGrayImage::ConstPtr gray);
    
    static KinectRgbImage::Ptr          LoadRgbFromFile(const std::string& file);
    static KinectRawDepthImage::Ptr     LoadDepthFromFile(const std::string& file);
    
    CLASS_DEF_PYEXPORT;
};

} // end of namespace utils
} // end of namespace KSRobot

#endif //KSROBOT_COMMON_KINECT_KINECTIMAGE_H
