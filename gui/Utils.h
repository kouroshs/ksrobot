#ifndef UTILS_H
#define UTILS_H

#include <utils/ProgramOptions.h>
#include <QSize>
#include <QRect>
#include <QMargins>
#include <QPoint>
#include <QString>
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
    static QSize                ReadSize(KSRobot::utils::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QSize& defaultVal);
    
    static void                 WriteSize(KSRobot::utils::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QSize& sz);
    
    static QRect                ReadRect(KSRobot::utils::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QRect& defaultVal);
    
    static void                 WriteRect(KSRobot::utils::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QRect& rect);
    
    static QMargins             ReadMargins(KSRobot::utils::ProgramOptions::Ptr po,
                                         const std::string& name,
                                         const QMargins& defaultVal);
    
    static void                 WriteMargins(KSRobot::utils::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QMargins& rect);
    
    static QPoint               ReadPoint(KSRobot::utils::ProgramOptions::Ptr po,
                                          const std::string& name,
                                          const QPoint& defVal);
    
    static void                 WritePoint(KSRobot::utils::ProgramOptions::Ptr po,
                                           const std::string& name,
                                           const QPoint& point);
    
    static QString              IsometryToString(const Eigen::Isometry3d& iso);

};

} // end namespace gui
} // end namespace KSRobot

#endif // UTILS_H
