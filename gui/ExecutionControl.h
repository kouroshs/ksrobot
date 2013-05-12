#ifndef KINECTDATACONTROL_H
#define KINECTDATACONTROL_H

#include <utils/ProgramOptions.h>
#include <gui/KWidgetBase.h>

#include <gui/ui_ExecutionControl.h>

namespace KSRobot
{
namespace gui
{

class ExecControlData
{
public:
    struct
    {
        bool            GetFromDevice;
        std::string     Source;
    } Kinect;
    
    struct
    {
        bool            Enable;
    } Fovis;
    
    struct
    {
        bool            Enable;
    } iSAM;
    
    struct
    {
        bool            Enable;
    } OctoMap;
    
    struct
    {
        bool            Enable;
    } OMPL;
    
    struct
    {
        bool            Enable;
    } Comm;
    
    struct
    {
        bool            ViewRGBD;
        bool            ViewPCL;
        bool            ViewFovis;
        bool            ViewOctoMap;
        bool            ViewOMPL;
    } GUI;
    
    void                CheckConsistancy();
    
    void                EnableFovis(bool enable);
    void                EnableSAM(bool enable);
    void                EnableOctoMap(bool enable);
    void                EnableOMPL(bool enable);
    void                EnableComm(bool enable);
};

class ExecutionControl : public gui::KWidgetBase
{
    Q_OBJECT
public:
    explicit ExecutionControl(QWidget* parent = 0, Qt::WindowFlags f = 0);
    virtual ~ExecutionControl();
    
    virtual void                                InitControl(utils::ProgramOptions::Ptr po, bool fromControlName = true);

signals:
    void                                        OnStart(const ExecControlData& data);
    void                                        OnStop();
    
private:
    bool                                        UpdateUIFromData();
    bool                                        UpdateDataFromUI();
private slots:
    void on_mBtnStart_clicked();
    void on_mBtnStop_clicked();
    void on_mRdBtnKinectDevice_clicked();
    void on_mRdBtnKinectFile_clicked();
    void on_mBtnBrowseKinect_clicked();
    void on_mChkEnableFovis_clicked();
    void on_mChkEnableSAM_clicked();
    void on_mChkEnableOctomap_clicked();
    void on_mChkEnableOMPL_clicked();
    void on_mChkEnableComm_clicked();
    void on_mChkViewRGBD_clicked();
    void on_mChkViewPointCloud_clicked();
    void on_mChkViewFovisResults_clicked();
    void on_mChkViewOctomap_clicked();
    void on_mChkViewOMPL_clicked();
private:
    Ui::ExecutionControl*                       mUI;
    ExecControlData                             mData;
};

} // end namespace gui
} // end namespace KSRobot

#endif // KINECTDATACONTROL_H
