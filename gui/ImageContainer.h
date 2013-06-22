#ifndef IMAGECONTAINER_H
#define IMAGECONTAINER_H

#include <gui/KWidgetBase.h>
#include <common/ProgramOptions.h>
#include <QImage>

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
    
    void                DrawImage(const QImage& img);
protected:
    virtual void        paintEvent(QPaintEvent* evt);
    
    QImage              mImage;
};

} // end namespace gui
} // end namespace KSRobot

#endif // IMAGECONTAINER_H
