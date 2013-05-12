#ifndef IMAGECONTAINER_H
#define IMAGECONTAINER_H

#include <gui/KWidgetBase.h>
#include <utils/ProgramOptions.h>

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
    
    virtual void InitControl(utils::ProgramOptions::Ptr po, bool fromControlName = true);
    
    virtual QSize minimumSizeHint() const;
    virtual QSize sizeHint() const;
};

} // end namespace gui
} // end namespace KSRobot

#endif // IMAGECONTAINER_H
