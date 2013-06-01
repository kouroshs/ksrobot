#include <gui/StatisticsContainer.h>
#include <gui/Utils.h>
#include <QFormLayout>

namespace KSRobot
{
namespace gui
{

StatisticsContainer::StatisticsContainer(QWidget* parent, Qt::WindowFlags f): KWidgetBase(parent, f)
{
    mNameMap[LR_MOTION] = "Fovis Motion: ";
    mNameMap[LR_POSITION] = "Fovis Position: ";
    mNameMap[LR_CYCLE] = "Fovis Cycle: ";
    mNameMap[LR_TIMESTAMP] = "Fovis Timestamp: ";
    mNameMap[LR_FOVIS_STATUS] = "Fovis status: ";
    mNameMap[LR_SUCCESS]= "% fovis::SUCCESS: ";
    mNameMap[LR_NO_DATA] = "% fovis::NO_DATA: ";
    mNameMap[LR_INSUFFICIENT_INLIERS] = "% fovis::INSUFFICIENT_INLIERS: ";
    mNameMap[LR_OPTIMIZATION_FAILURE] = "% fovis::OPTIMIZATION_FAILURE: ";
    mNameMap[LR_REPROJECTION_ERROR] = "% fovis::REPROJECTION_ERROR: ";
    
    for(int i = (int)LR_BEGIN; i < (int)LR_END; i++)
    {
        QLabel* lbl = new QLabel(this);
        lbl->setText(tr("                     "));
        mLabelsMap.insert((LabelRep)i, lbl);
    }
    
    QFormLayout* layout = new QFormLayout(this);
    for(int i = (int)LR_BEGIN; i < (int)LR_END; i++)
        layout->insertRow(i, mNameMap[(LabelRep)i], mLabelsMap[(LabelRep)i]);
    setLayout(layout);
}

StatisticsContainer::~StatisticsContainer()
{
}

void StatisticsContainer::InitControl(common::ProgramOptions::Ptr po)
{
    KSRobot::gui::KWidgetBase::InitControl(po);
}


// void StatisticsContainer::OnFovisDone(const MotionEstimateResult& me, const FovisStatistics& stat)
// {
//     mLabelsMap[LR_MOTION]->setText(Utils::IsometryToString(me.Motion));
//     mLabelsMap[LR_POSITION]->setText(Utils::IsometryToString(me.Pos));
//     mLabelsMap[LR_CYCLE]->setText(QString::number(me.Cycle));
//     mLabelsMap[LR_TIMESTAMP]->setText(QString::number(me.TimeStamp, 'g', 16));
//     mLabelsMap[LR_FOVIS_STATUS]->setText(fovis::MotionEstimateStatusCodeStrings[me.Status]);
//     mLabelsMap[LR_SUCCESS]->setText(QString::number(stat.GetPercentage(fovis::SUCCESS)));
//     mLabelsMap[LR_NO_DATA]->setText(QString::number(stat.GetPercentage(fovis::NO_DATA)));
//     mLabelsMap[LR_INSUFFICIENT_INLIERS]->setText(QString::number(stat.GetPercentage(fovis::INSUFFICIENT_INLIERS)));
//     mLabelsMap[LR_OPTIMIZATION_FAILURE]->setText(QString::number(stat.GetPercentage(fovis::OPTIMIZATION_FAILURE)));
//     mLabelsMap[LR_REPROJECTION_ERROR]->setText(QString::number(stat.GetPercentage(fovis::REPROJECTION_ERROR)));
// }


} // end namespace gui
} // end namespace KSRobot
