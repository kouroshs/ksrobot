/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <gui/PointCloudViewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

namespace KSRobot
{
namespace gui
{

class PointCloudViewer::Impl
{
public:
    Impl() : Vis("Visualizer", false) {;}
    pcl::visualization::PCLVisualizer           Vis;
};

PointCloudViewer::PointCloudViewer(QWidget* parent, Qt::WindowFlags f): QVTKWidget(parent, f)
{
    mImpl = new PointCloudViewer::Impl;
    mImpl->Vis.addPointCloud(common::KinectPointCloud::Ptr(new common::KinectPointCloud));
    Eigen::Affine3f trans;
    trans.setIdentity();
    trans.rotate(Eigen::AngleAxisf(3.14159265, Eigen::Vector3f(0, 0, 1)));
    
    mImpl->Vis.addCoordinateSystem(1.0, trans);
    mImpl->Vis.setBackgroundColor(0, 0, 0);
    SetRenderWindow(mImpl->Vis.getRenderWindow().GetPointer());
}

PointCloudViewer::~PointCloudViewer()
{
    delete mImpl;
}

QSize PointCloudViewer::minimumSizeHint() const
{
    return QSize(500, 500);
}

QSize PointCloudViewer::sizeHint() const
{
    return minimumSizeHint();
}

void PointCloudViewer::UpdataPointCloud(common::KinectPointCloud::ConstPtr pc)
{
    mImpl->Vis.updatePointCloud(pc);
    update();
}


} // end namespace gui
} // end namespace KSRobot
