#include <gui/ImageContainer.h>
#include <QResizeEvent>
#include <QPainter>
#include <QPaintEvent>
#include <iostream>

using namespace KSRobot::gui;

ImageContainer::ImageContainer(QWidget* parent, Qt::WindowFlags f): KWidgetBase(parent, f)
{
}

ImageContainer::~ImageContainer()
{
}

void ImageContainer::InitControl(KSRobot::common::ProgramOptions::Ptr po)
{
    KSRobot::gui::KWidgetBase::InitControl(po);
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

void ImageContainer::DrawImage(const QImage& img)
{
    mImage = img;
    update();
}

void ImageContainer::paintEvent(QPaintEvent* evt)
{
    QWidget::paintEvent(evt);
    QPainter p(this);
    p.drawImage(rect(), mImage, mImage.rect());
}
