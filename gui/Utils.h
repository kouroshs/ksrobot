#ifndef UTILS_H
#define UTILS_H

#include <common/ProgramOptions.h>
#include <common/KinectImage.h>
#include <QSize>
#include <QRect>
#include <QMargins>
#include <QPoint>
#include <QString>
#include <QImage>
#include <Eigen/Geometry>

//#ifdef USE_SHORT_NAMES
#define KQT_OBJECT_NAME(obj) QString(#obj)
//#else // USE_COMPLETE_NAME
//#define KQT_OBJECT_NAME(obj) this->objectName() + "." #obj
//#endif


#define SET_QTOBJECTNAME(obj) (obj)->setObjectName(KQT_OBJECT_NAME(obj));

namespace KSRobot
{
namespace gui
{

class Utils
{
public:
    static void                 RegisterDefaultQtTypes();
    
    static QSize                ReadSize(KSRobot::common::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QSize& defaultVal);
    
    static void                 WriteSize(KSRobot::common::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QSize& sz);
    
    static QRect                ReadRect(KSRobot::common::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QRect& defaultVal);
    
    static void                 WriteRect(KSRobot::common::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QRect& rect);
    
    static QMargins             ReadMargins(KSRobot::common::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QMargins& defaultVal);
    
    static void                 WriteMargins(KSRobot::common::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QMargins& rect);
    
    static QPoint               ReadPoint(KSRobot::common::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QPoint& defVal);
    
    static void                 WritePoint(KSRobot::common::ProgramOptions::Ptr po,
                                           const std::string& name,
                                           const QPoint& point);
    
    static QString              IsometryToString(const Eigen::Isometry3d& iso);

    static QImage               ConvertToQImage(common::KinectRgbImage::Ptr rgb);
    static QImage               ConvertToQImage(common::KinectRawDepthImage::Ptr depth);
};

} // end namespace gui
} // end namespace KSRobot

#endif // UTILS_H
