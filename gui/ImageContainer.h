#ifndef IMAGECONTAINER_H
#define IMAGECONTAINER_H

#include <gui/KWidgetBase.h>
#include <common/ProgramOptions.h>
#include <QImage>
#include <boost/graph/graph_concepts.hpp>

namespace KSRobot
{
namespace gui
{

class ImageContainer : public KWidgetBase
{
    Q_OBJECT
public:
    explicit ImageContainer(QWidget* parent = 0, Qt::WindowFlags f = 0);
    virtual ~ImageContainer();
    
    virtual void        InitControl(common::ProgramOptions::Ptr po);
    
    virtual QSize       minimumSizeHint() const;
    virtual QSize       sizeHint() const;

public slots:    
    void                OnRGBD(const QImage& rgb, const QImage& depth);
protected:
    virtual void        paintEvent(QPaintEvent* evt);
    
    QImage              mRgb;
    QImage              mDepth;
};

} // end namespace gui
} // end namespace KSRobot

#endif // IMAGECONTAINER_H
