#include "Utils.h"
#include <stdio.h>

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

void Utils::RegisterDefaultQtTypes()
{
    ADD_TYPE(QString);
    ADD_TYPE(QPoint);
    ADD_TYPE(QMargins);
    ADD_TYPE(QRect);
    ADD_TYPE(QSize);
}

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