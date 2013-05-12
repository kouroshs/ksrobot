#include "Utils.h"
#include <stdio.h>

namespace KSRobot
{
namespace gui
{

using utils::ProgramOptions;
    
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



} // end namespace gui
} // end namespace KSRobot