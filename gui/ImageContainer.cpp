#include <gui/ImageContainer.h>
#include <QResizeEvent>
#include <iostream>

using namespace KSRobot::gui;

ImageContainer::ImageContainer(QWidget* parent, Qt::WindowFlags f): KWidgetBase(parent, f)
{
}

ImageContainer::~ImageContainer()
{
}

void ImageContainer::InitControl(KSRobot::utils::ProgramOptions::Ptr po, bool fromControlName)
{
    KSRobot::gui::KWidgetBase::InitControl(po, fromControlName);
    setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
}

QSize ImageContainer::minimumSizeHint() const
{
    return QSize(320, 240);
}

QSize ImageContainer::sizeHint() const
{
    return minimumSizeHint();
}
