#ifndef STATISTICSCONTAINER_H
#define STATISTICSCONTAINER_H

#include <common/ProgramOptions.h>
#include <gui/KWidgetBase.h>
#include <QLabel>
#include <QMap>
//TODO: Add this widget plus an action to MainWindow

namespace KSRobot
{
namespace gui
{

class StatisticsContainer : public KWidgetBase
{
    Q_OBJECT
public:
    explicit StatisticsContainer(QWidget* parent = 0, Qt::WindowFlags f = 0);
    virtual ~StatisticsContainer();
    
    virtual void InitControl(common::ProgramOptions::Ptr po);
    
private:
    enum LabelRep
    {
        LR_BEGIN = 0,
        LR_MOTION = LR_BEGIN,
        LR_POSITION,
        LR_CYCLE,
        LR_TIMESTAMP,
        
        //Fovis enum
        LR_FOVIS_STATUS,
        LR_SUCCESS,
        LR_NO_DATA,
        LR_INSUFFICIENT_INLIERS,
        LR_OPTIMIZATION_FAILURE,
        LR_REPROJECTION_ERROR,
        
        LR_END
    };
    
    QMap<LabelRep, QLabel*>                             mLabelsMap;
    QMap<LabelRep, QString>                             mNameMap;
    
};

} // end namespace gui
} // end namespace KSRobot

#endif // STATISTICSCONTAINER_H
