#include <common/KinectImage.h>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>

#include <common/Timer.h>
#include <iostream>
#include <fstream>

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
    std::string::size_type pos = file.find_last_of('.');
    KinectRgbImage::Ptr ret(new KinectRgbImage);
    if( pos == std::string::npos || file.substr(pos + 1) == "raw" )
    {
        std::ifstream input(file.c_str(), std::ios::in);
        boost::int32_t w, h;
        input >> w >> h;
        ret->Create(w, h);
        input.read((char*)ret->GetArray().data(), ret->GetNumElements() * sizeof(KinectRgbImage::BaseDataType));
        input.close();
    }
    else
    {
        gil::rgb8_image_t image;
        gil::png_read_image(file, image);
        gil::rgb8_view_t view = gil::view(image);
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
    }
    return ret;
}

KinectRawDepthImage::Ptr KinectImageDiskIO::LoadDepthFromFile(const std::string& file)
{
    std::string::size_type pos = file.find_last_of('.');
    KinectRawDepthImage::Ptr ret(new KinectRawDepthImage);
    if( pos == std::string::npos || file.substr(pos + 1) == "raw" )
    {
        std::ifstream input(file.c_str());
        boost::int32_t w, h;
        input >> w >> h;
        ret->Create(w, h);
        input.read((char*)ret->GetArray().data(), ret->GetNumElements() * sizeof(KinectRawDepthImage::BaseDataType));
        input.close();
    }
    else
    {
        gil::gray16_image_t image;
        gil::png_read_image(file, image);
        gil::gray16_view_t view = gil::view(image);
        
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
    }
    return ret;
}

void KinectImageDiskIO::SaveToFileRgb(const std::string& file, KinectRgbImage::ConstPtr rgb)
{
    std::string::size_type pos = file.find_last_of('.');
    if( pos == std::string::npos || file.substr(pos + 1) == "raw" )
    {
        std::ofstream out(file.c_str());
        boost::int32_t w = rgb->GetWidth(), h = rgb->GetHeight(); // this is to ensure size consistancy across platforms
        out << w << h;
        out.write((char*)rgb->GetArray().data(), rgb->GetNumElements() * sizeof(KinectRgbImage::BaseDataType));
        out.close();
    }
    else
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
}

void KinectImageDiskIO::SaveToFileDepth(const std::string& file, KinectRawDepthImage::ConstPtr depth)
{
    std::string::size_type pos = file.find_last_of('.');
    if( pos == std::string::npos || file.substr(pos + 1) == "raw" )
    {
        std::ofstream out(file.c_str());
        boost::int32_t w = depth->GetWidth(), h = depth->GetHeight(); // this is to ensure size consistancy across platforms
        out << w << h;
        out.write((char*)depth->GetArray().data(), depth->GetNumElements() * sizeof(KinectRawDepthImage::BaseDataType));
        out.close();
    }
    else
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
}

};
};
