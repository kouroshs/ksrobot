#include <gui/Utils.h>
#include <common/KinectInterface.h>
#include <stdio.h>

#include <QMetaType>
#include <QColor>

#include <pcl/point_cloud.h> //basic pcl includes
#include <pcl/point_types.h>

namespace KSRobot
{
namespace gui
{

using common::ProgramOptions;

template<class X>
class QtTypeInterface : public ProgramOptions::UserTypeInterface
{
public:
    virtual ~QtTypeInterface() {;}
    
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        throw std::exception("(QtTypeInterface::Put) Unknown type");
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        throw std::exception("(QtTypeInterface::Get) Unknown type");
    }
    
};

template <>
class QtTypeInterface<QSize> : public ProgramOptions::UserTypeInterface
{
public:
    virtual ~QtTypeInterface() {;}
    
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        QSize sz = boost::any_cast<QSize>(value);
        po->PutInt("Width", sz.width());
        po->PutInt("Height", sz.height());
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        QSize sz;
        sz.setWidth(po->GetInt("Width"));
        sz.setHeight(po->GetInt("Height"));
        return sz;
    }
};

template<>
class QtTypeInterface<QRect> : public ProgramOptions::UserTypeInterface
{
public:
    virtual ~QtTypeInterface() {;}
    
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        QRect r = boost::any_cast<QRect>(value);
        po->PutInt("Left", r.left());
        po->PutInt("Top", r.top());
        po->PutInt("Width", r.width());
        po->PutInt("Height", r.height());
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        QRect r;
        r.setLeft(po->GetInt("Left"));
        r.setTop(po->GetInt("Top"));
        r.setWidth(po->GetInt("Width"));
        r.setHeight(po->GetInt("Height"));
        return r;
    }
};

template<>
class QtTypeInterface<QMargins> : public ProgramOptions::UserTypeInterface
{
public:
    virtual ~QtTypeInterface() {;}
    
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        QMargins r = boost::any_cast<QMargins>(value);
        po->PutInt("Left", r.left());
        po->PutInt("Top", r.top());
        po->PutInt("Right", r.right());
        po->PutInt("Bottom", r.bottom());
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        QMargins r;
        r.setLeft(po->GetInt("Left"));
        r.setTop(po->GetInt("Top"));
        r.setRight(po->GetInt("Right"));
        r.setBottom(po->GetInt("Bottom"));
        return r;
    }
};

template<>
class QtTypeInterface<QPoint> : public ProgramOptions::UserTypeInterface
{
public:
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        QPoint pt = boost::any_cast<QPoint>(value);
        po->PutInt("X", pt.x());
        po->PutInt("Y", pt.y());
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        QPoint pt;
        pt.setX(po->GetInt("X"));
        pt.setY(po->GetInt("Y"));
        return pt;
    }
};

template<>
class QtTypeInterface<QString> : public ProgramOptions::UserTypeInterface
{
public:
    virtual void Put(ProgramOptions::Ptr po, const boost::any& value)
    {
        QString str = boost::any_cast<QString>(value);
        po->PutNodeValue(str.toStdString());
    }
    
    virtual boost::any Get(ProgramOptions::Ptr po)
    {
        return QString(po->GetNodeValue().c_str());
    }
};

#define ADD_TYPE(T) ProgramOptions::AddUserType<T>(static_cast<ProgramOptions::UserTypeInterface*>(new QtTypeInterface<T>()))


//Q_DECLARE_METATYPE(common::KinectPointCloud::Ptr);

void Utils::RegisterDefaultQtTypes()
{
    ADD_TYPE(QString);
    ADD_TYPE(QPoint);
    ADD_TYPE(QMargins);
    ADD_TYPE(QRect);
    ADD_TYPE(QSize);
    
    qRegisterMetaType<common::KinectPointCloud::Ptr>("common::KinectPointCloud::Ptr");
    qRegisterMetaType<common::KinectPointCloud::ConstPtr>("common::KinectPointCloud::ConstPtr");}

QSize Utils::ReadSize(ProgramOptions::Ptr po, const std::string& name, const QSize& defaultVal)
{
    QSize ret;
    ProgramOptions::Ptr node = po->StartNode(name);
    ret.setWidth(node->GetInt("Width", defaultVal.width()));
    ret.setHeight(node->GetInt("Height", defaultVal.height()));
    return ret;
}

void Utils::WriteSize(ProgramOptions::Ptr po, const std::string& name, const QSize& sz)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    node->PutInt("Width", sz.width());
    node->PutInt("Height", sz.height());
}

QRect Utils::ReadRect(ProgramOptions::Ptr po, const std::string& name, const QRect& defaultVal)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    QRect rect;
    
    rect.setLeft(node->GetInt("Left", defaultVal.left()));
    rect.setTop(node->GetInt("Top", defaultVal.top()));
    rect.setWidth(node->GetInt("Width", defaultVal.width()));
    rect.setHeight(node->GetInt("Height", defaultVal.height()));
    
    return rect;
}

void Utils::WriteRect(ProgramOptions::Ptr po, const std::string& name, const QRect& rect)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    node->PutInt("Left", rect.left());
    node->PutInt("Top", rect.top());
    node->PutInt("Width", rect.width());
    node->PutInt("Height", rect.height());
}

QMargins Utils::ReadMargins(ProgramOptions::Ptr po, const std::string& name, const QMargins& defaultVal)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    QMargins ret;
    
    ret.setLeft(node->GetInt("Left", defaultVal.left()));
    ret.setTop(node->GetInt("Top", defaultVal.top()));
    ret.setRight(node->GetInt("Right", defaultVal.right()));
    ret.setBottom(node->GetInt("Bottom", defaultVal.bottom()));
    
    return ret;
}

void Utils::WriteMargins(ProgramOptions::Ptr po, const std::string& name, const QMargins& rect)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    node->PutInt("Left", rect.left());
    node->PutInt("Top", rect.top());
    node->PutInt("Right", rect.right());
    node->PutInt("Bottom", rect.bottom());
}

QPoint Utils::ReadPoint(ProgramOptions::Ptr po, const std::string& name, const QPoint& defVal)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    QPoint pt;
    pt.setX(node->GetInt("X", defVal.x()));
    pt.setY(node->GetInt("Y", defVal.y()));
    return pt;
}

void Utils::WritePoint(ProgramOptions::Ptr po, const std::string& name, const QPoint& point)
{
    ProgramOptions::Ptr node = po->StartNode(name);
    node->PutInt("X", point.x());
    node->PutInt("Y", point.y());
}

QString Utils::IsometryToString(const Eigen::Isometry3d& m)
{
    char result[80];
    memset(result, 0, sizeof(result));
    Eigen::Vector3d xyz = m.translation();
    Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
    snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", 
      xyz(0), xyz(1), xyz(2), 
      rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
    
    return QString(result);
}

QImage Utils::ConvertToQImage(KSRobot::common::KinectRgbImage::ConstPtr rgb)
{
    QImage qrgb(rgb->GetWidth(), rgb->GetHeight(), QImage::Format_RGB32);
    const common::KinectRgbImage::ArrayType& rgbArray = rgb->GetArray();
    
    for(int y = 0; y < rgb->GetHeight(); y++)
    {
        uchar* rgbPtr = qrgb.scanLine(y);
        size_t idxRgb = rgb->ScanLineIndex(y);
        for(int x = 0; x < rgb->GetWidth(); x++)
        {
            rgbPtr[0] = rgbArray[idxRgb];
            rgbPtr[1] = rgbArray[idxRgb + 1];
            rgbPtr[2] = rgbArray[idxRgb + 2];
            rgbPtr[3] = 0xFF;
            
            idxRgb = common::KinectRgbImage::NextIndexUnsafe(idxRgb);
            rgbPtr += 4;
        }
    }
    
    return qrgb;
}

QImage Utils::ConvertToQImage(KSRobot::common::KinectRawDepthImage::ConstPtr depth)
{
    QImage qdepth(depth->GetWidth(), depth->GetHeight(), QImage::Format_RGB32);
    const common::KinectRawDepthImage::ArrayType& depthArray = depth->GetArray();
    
    const float MaxDepth = 8000.0f;
    
    qdepth.fill(QColor(0xff,0xff,0xff));
    
    for(int y = 0; y < depth->GetHeight(); y++)
    {
        uchar* depthPtr = qdepth.scanLine(y);
        size_t idxDepth = depth->ScanLineIndex(y);
        for(int x = 0; x < depth->GetWidth(); x++)
        {
            unsigned char r = 0xFF, g = 0xFF, b = 0xFF;
            if( depthArray[idxDepth] != 0 )
            {
                float val = (float)depthArray[idxDepth] / MaxDepth;
                if( val > 1.0f )
                    val = 1.0f;
                
                if( val < 0.25f )
                {
                    r = 0;
                    g = (4 * val) * 255;
                }
                else if( val < 0.5f )
                {
                    r = 0;
                    b = (1 + 4 * (0.25f - val)) * 255;
                }
                else if( val < 0.75f )
                {
                    r = 4 * (val - 0.5f) * 255;
                    b = 0;
                }
                else
                {
                    g = (1 + 4 * (0.75f - val)) * 255;
                    b = 0;
                }
            }
            else
            {
                r = g = b = 0;
            }
            depthPtr[0] = r;
            depthPtr[1] = g;
            depthPtr[2] = b;
            depthPtr[3] = 0xFF;
                
            idxDepth = common::KinectRawDepthImage::NextIndexUnsafe(idxDepth);
            depthPtr += 4;
        }
    }
    
    return qdepth;
}



} // end namespace gui
} // end namespace KSRobot