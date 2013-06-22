#ifndef KINECTDATACONTROL_H
#define KINECTDATACONTROL_H

#include <common/ProgramOptions.h>
#include <common/SettingsBinder.h>
#include <common/ExecCtrlData.h>
#include <gui/KWidgetBase.h>
#include <gui/ui_ExecutionControl.h>


namespace KSRobot
{
namespace gui
{

class ExecutionControl : public gui::KWidgetBase
{
    Q_OBJECT
public:
    explicit ExecutionControl(QWidget* parent = 0, Qt::WindowFlags f = 0);
    virtual ~ExecutionControl();
    
    virtual void                                InitControl(common::ProgramOptions::Ptr po);

signals:
    void                                        OnStart(const common::ExecCtrlData& data);
    void                                        OnStop();

public slots:
    void                                        GuiStop();
    
private:
    void                                        BindSettings();
    void                                        ReadSettings();
    void                                        SaveSettings();
    bool                                        UpdateUIFromData();
    bool                                        UpdateDataFromUI();
    
    void                                        BindValueToName(const std::string& name, QAbstractButton* btn);
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
    Ui::ExecutionControl*                        mUI;
    common::ExecCtrlData                         mData;
    common::SettingsBinder                       mBinderFns;
    common::SettingsBinder                       mBinderVars;
};

} // end namespace gui
} // end namespace KSRobot

#endif // KINECTDATACONTROL_H
