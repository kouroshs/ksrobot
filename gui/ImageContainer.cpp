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
    return QSize(320 * 2, 240);
}

QSize ImageContainer::sizeHint() const
{
    return minimumSizeHint();
}

void ImageContainer::OnRGBD(const QImage& rgb, const QImage& depth)
{
    mRgb = rgb;
    mDepth = depth;
    update();
}

void ImageContainer::paintEvent(QPaintEvent* evt)
{
    QWidget::paintEvent(evt);
    
    QPainter p(this);
    int w = width();
    int h = height();
    p.drawImage(QRect(0, 0, w / 2, h), mRgb, mRgb.rect());
    p.drawImage(QRect(w / 2 + 1, 0, w / 2, h), mDepth, mDepth.rect());
}
