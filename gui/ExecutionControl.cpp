#include "ExecutionControl.h"

#include <QVBoxLayout>
#include <QRect>
#include "Utils.h"
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

using namespace KSRobot;
using namespace gui;
using namespace utils;

void ExecControlData::EnableComm(bool enable)
{
//     if( !OMPL.Enable )
//         throw std::runtime_error("Cannot enable communications while OMPL is disabled.");
    Comm.Enable = enable; // for now no dependency
}

void ExecControlData::EnableFovis(bool enable)
{
//     if( !enable && (OctoMap.Enable || OMPL.Enable || iSAM.Enable) )
//         throw std::runtime_error("Cannot disable Fovis. Cause : OctoMap|OMPL|iSAM");
    Fovis.Enable = enable;
}

void ExecControlData::EnableSAM(bool enable)
{
    //NOTE: For now SAM is independent
//     if( enable && !Fovis.Enable )
//         throw std::runtime_error("Cannot enable iSAM while Fovis is disabled.");
    iSAM.Enable = enable;
}

void ExecControlData::EnableOctoMap(bool enable)
{
//     if( enable && !Fovis.Enable )
//         throw std::runtime_error("Cannot enable OctoMap while Fovis is disabled");
//     
//     if( !enable && OMPL.Enable )
//         throw std::runtime_error("Cannot disable OctoMap while OMPL is set.");
//     
    OctoMap.Enable = enable;
}

void ExecControlData::EnableOMPL(bool enable)
{
//     if( !Fovis.Enable || !OctoMap.Enable )
//         throw std::runtime_error("Cannot enable OMPLE. Cause: Fovis|OctoMap");
//     
    OMPL.Enable = enable;
}

void ExecControlData::CheckConsistancy()
{
    if( !Fovis.Enable )
    {
        if( OctoMap.Enable )
            throw std::runtime_error("Cannot enable OctoMap while Fovis is disabled.");
        if( iSAM.Enable )
            throw std::runtime_error("Cannot enable iSAM while Fovis is disabled.");
        if( Comm.Enable )
            throw std::runtime_error("Cannot enable Communications while Fovis is disabled.");
    }
    
    if( !OctoMap.Enable )
    {
        if( OMPL.Enable )
            throw std::runtime_error("Cannot enable OMPL while OctoMap is disabled.");
    }

    if( !iSAM.Enable )
    {
        // For now iSAM is independent.
    }
    
    if( !OMPL.Enable )
    {
        if( Comm.Enable )
            throw std::runtime_error("Cannot enable Communications while OMPL is disabled.");
    }
}


ExecutionControl::ExecutionControl(QWidget* parent, Qt::WindowFlags f) : KWidgetBase(parent, f)
{
    qRegisterMetaType<ExecControlData>("ExecControlData");
}

ExecutionControl::~ExecutionControl()
{
}

void ExecutionControl::InitControl(ProgramOptions::Ptr po)
{
    KWidgetBase::InitControl(po);

    mUI = new Ui::ExecutionControl();
    //NOTE: Because the ui is very big, I'm going to turn a blind eye on naming conventions inside this class,
    // so I don't have to change hundreds of places after a simple gui reconstruction inside qtcreator
    
    mUI->setupUi(this);
    mUI->retranslateUi(this);
    
    if( mGuiPO->GetString("DefaultSource", std::string("file")) == "kinect" )
        mUI->mRdBtnKinectDevice->setChecked(true);
    else
        mUI->mRdBtnKinectFile->setChecked(true);
}

bool ExecutionControl::UpdateUIFromData()
{
    //TODO: Implement this.
    return false;
}

bool ExecutionControl::UpdateDataFromUI()
{
    //TODO: Implement this.
    try
    {
        // Update Kinect
        mData.Kinect.GetFromDevice = mUI->mRdBtnKinectDevice->isChecked();
        mData.Kinect.Source = mUI->mKinectDevPath->text().toStdString();
        // Update Fovis
        mData.EnableFovis(mUI->mChkEnableFovis->isChecked());
        // Update iSAM
        mData.EnableSAM(mUI->mChkEnableSAM->isChecked());
        // Update OctoMap
        mData.EnableOctoMap(mUI->mChkEnableOctomap->isChecked());
        // Update OMPL
        mData.EnableOMPL(mUI->mChkEnableOMPL->isChecked());
        // Update Commserver
        mData.EnableComm(mUI->mChkEnableComm->isChecked());
        
        // Update GUI settings
        mData.GUI.ViewFovis = mUI->mChkViewFovisResults->isChecked();
        mData.GUI.ViewOctoMap = mUI->mChkViewOctomap->isChecked();
        mData.GUI.ViewOMPL = mUI->mChkViewOMPL->isChecked();
        mData.GUI.ViewPCL = mUI->mChkViewPointCloud->isChecked();
        mData.GUI.ViewRGBD = mUI->mChkViewRGBD->isChecked();
        
        mData.CheckConsistancy();
    }
    catch(std::runtime_error& re)
    {
        QMessageBox::critical(this, "ExecutionControl::UpdateDataFromUI", QString("Invalid settings: ") + re.what());
        return false;
    }
    return true;
}

void ExecutionControl::on_mBtnStart_clicked()
{
    if( !UpdateDataFromUI() )
        return;
    
    mUI->mTabSettings->setEnabled(false);
    mUI->mBtnStart->setEnabled(false);
    mUI->mBtnStop->setEnabled(true);
    
    emit OnStart(mData);
}

void ExecutionControl::on_mBtnStop_clicked()
{
    emit OnStop();
    
    mUI->mTabSettings->setEnabled(true);
    mUI->mBtnStart->setEnabled(true);
    mUI->mBtnStop->setEnabled(false);
}

void ExecutionControl::on_mRdBtnKinectDevice_clicked()
{
}

void ExecutionControl::on_mRdBtnKinectFile_clicked()
{
}

void ExecutionControl::on_mBtnBrowseKinect_clicked()
{
    QString dir;
    if( mUI->mRdBtnKinectFile->isChecked() )
    {
        std::string defDir = mGuiPO->GetString("DefaultDirPath",
                                std::string("/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/"));
        
        dir = QFileDialog::getExistingDirectory(this, tr("Select the path to dataset"),
                                                defDir.c_str());
    }
    else
    {
        qDebug() << "(ExecutionControl::on_mBtnBrowseKinect_clicked) Not implemented for kinect device.";
    }
}

void ExecutionControl::on_mChkEnableFovis_clicked()
{
}

void ExecutionControl::on_mChkEnableSAM_clicked()
{
}

void ExecutionControl::on_mChkEnableOctomap_clicked()
{
}

void ExecutionControl::on_mChkEnableOMPL_clicked()
{
}

void ExecutionControl::on_mChkEnableComm_clicked()
{
}

void ExecutionControl::on_mChkViewRGBD_clicked()
{
}

void ExecutionControl::on_mChkViewPointCloud_clicked()
{
}

void ExecutionControl::on_mChkViewFovisResults_clicked()
{
}

void ExecutionControl::on_mChkViewOctomap_clicked()
{
}

void ExecutionControl::on_mChkViewOMPL_clicked()
{
}
