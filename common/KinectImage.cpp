#include <common/KinectImage.h>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>

#include <common/Timer.h>
#include <iostream>

using std::cout;
using std::endl;
using std::flush;

namespace KSRobot
{
namespace common
{

namespace gil = boost::gil;

KinectRgbImage::Ptr KinectImageDiskIO::LoadRgbFromFile(const std::string& file)
{
    gil::rgb8_image_t image;
    gil::png_read_image(file, image);
    gil::rgb8_view_t view = gil::view(image);
    KinectRgbImage::Ptr ret(new KinectRgbImage);
    ret->Create(image.width(), image.height());
    KinectRgbImage::ArrayType& array = ret->GetArray();
    size_t index = 0;
    for(int i = 0; i < image.height(); i++)
    {
        gil::rgb8_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < image.width(); j++)
        {
            gil::rgb8_pixel_t& pix = iter[j];
            array[index++] = pix[0];
            array[index++] = pix[1];
            array[index++] = pix[2];
        }
    }
    return ret;
}

KinectRawDepthImage::Ptr KinectImageDiskIO::LoadDepthFromFile(const std::string& file)
{
    gil::gray16_image_t image;
    gil::png_read_image(file, image);
    gil::gray16_view_t view = gil::view(image);
    
    KinectRawDepthImage::Ptr ret(new KinectRawDepthImage);
    ret->Create(image.width(), image.height());
    KinectRawDepthImage::ArrayType& array = ret->GetArray();
    size_t index = 0;
    for(int i = 0; i < image.height(); i++)
    {
        gil::gray16_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < image.width(); j++)
        {
            array[index++] = iter[j] / 5;
        }
    }
    return ret;
}

void KinectImageDiskIO::SaveToFileRgb(const std::string& file, KinectRgbImage::ConstPtr rgb)
{
    gil::rgb8_image_t::point_t dims(rgb->GetWidth(), rgb->GetHeight());
    gil::rgb8_image_t image(dims);
    gil::rgb8_view_t view = gil::view(image);
    const KinectRgbImage::ArrayType& array = rgb->GetArray();
    size_t index = 0;
    for(int i = 0; i < image.height(); i++)
    {
        gil::rgb8_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < image.width(); j++)
        {
            gil::rgb8_pixel_t& pix = iter[j];
            pix[0] = array[index++];
            pix[1] = array[index++];
            pix[2] = array[index++];
        }
    }
    gil::png_write_view(file, view);
}

void KinectImageDiskIO::SaveToFileDepth(const std::string& file, KinectRawDepthImage::ConstPtr depth)
{
    gil::gray16_image_t::point_t dims(depth->GetWidth(), depth->GetHeight());
    gil::gray16_image_t image(dims);
    gil::gray16_view_t view = gil::view(image);
    const KinectRawDepthImage::ArrayType& array = depth->GetArray();
    size_t index = 0;
    for(int i = 0; i < image.height(); i++)
    {
        gil::gray16_view_t::x_iterator iter = view.row_begin(i);
        for(int j = 0; j < image.width(); j++)
            iter[j] = array[index++] * 5;
    }
    gil::png_write_view(file, view);
}

};
};
