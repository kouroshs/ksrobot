#include <roboctrl/RobXControlDialog.h>
#include <roboctrl/ui_RobXControlDialog.h>

#include <QMessageBox>
#include <QTimer>

#include <iostream>

#if 1
#define ENABLE      ui->tabControlLevel->setEnabled(true)
#define DISABLE     ui->tabControlLevel->setEnabled(false)
#else
#define ENABLE
#define DISABLE
#endif

namespace KSRobot
{
namespace roboctrl
{

RobXControlDialog::RobXControlDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobXCtrlDlg),
    mCtrl(0)
{
    ui->setupUi(this);
    ui->deviceLineEdit->setText("/dev/ttyUSB0");
    DISABLE;
    mEncoderUpdateTimer = new QTimer(this);
    mEncoderUpdateTimer->setInterval(2000);
    connect(mEncoderUpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateMotorInfo()));
}

RobXControlDialog::~RobXControlDialog()
{
    delete ui;
}

void RobXControlDialog::Connect(const QString& device)
{
    mCtrl = new RobXControl(this);
    try
    {
        DISABLE;
        mCtrl->Open(device);
        ENABLE;
        mEncoderUpdateTimer->start();
        //std::cout << "After timer call\n" << std::flush;
    }
    catch(std::exception& ex)
    {
        QMessageBox::critical(this, "Open Device Error", ex.what());
    }
    catch(...)
    {
        QMessageBox::critical(this, "Open Device Error", "Unknown Error");
    }
}

void RobXControlDialog::Disconnect()
{
    mEncoderUpdateTimer->stop();
    if( mCtrl )
    {
        try
        {
            mCtrl->Stop();
        }
        catch(std::exception& ex)
        {
            QMessageBox::critical(this, "Close Device Error", ex.what());
        }
        catch(...)
        {
            QMessageBox::critical(this, "Close Device Error", "Unknown Error");
        }
        
        delete mCtrl;
    }
}


void RobXControlDialog::on_btnConnectDevice_clicked()
{
    Disconnect();
    Connect(ui->deviceLineEdit->text());
}

void RobXControlDialog::on_btnStop_clicked()
{
    Stop();
}

void RobXControlDialog::on_btnForward_clicked()
{
    mCtrl->Forward(ui->sldrMoveAmount->value(), ui->sldrMoveSpeed->value());
}

void RobXControlDialog::on_btnBackward_clicked()
{
    mCtrl->Backward(ui->sldrMoveAmount->value(), ui->sldrMoveSpeed->value());
}

void RobXControlDialog::on_btnTurnRight_clicked()
{
    mCtrl->TurnRight(ui->sldrTurnAmount->value(), ui->sldrTurnSpeed->value());
}

void RobXControlDialog::on_btnTurnLeft_clicked()
{
    mCtrl->TurnLeft(ui->sldrTurnAmount->value(), ui->sldrTurnSpeed->value());
}

void RobXControlDialog::on_btnStop2_clicked()
{
    Stop();
}

void RobXControlDialog::on_chkSyncMotors_clicked(bool checked)
{
    if( checked )
    {
//        ui->sliderMotor2->setEnabled(false);
        ui->sliderMotor2->hide();
        ui->lblMotor2->hide();
        ui->sliderMotor2->setValue(ui->sliderMotor1->value());
    }
    else
    {
//        ui->sliderMotor2->setEnabled(true);
        ui->sliderMotor2->show();
        ui->lblMotor2->show();
    }
}

void RobXControlDialog::on_sliderMotor1_valueChanged(int value)
{
    ui->lblMotor1->setText(QString::number(value));
    if( ui->chkSyncMotors->isChecked() )
        ui->sliderMotor2->setValue(value);
}

void RobXControlDialog::on_sliderMotor2_valueChanged(int value)
{
    ui->lblMotor2->setText(QString::number(value));
}

void RobXControlDialog::on_tabControlLevel_currentChanged(int index)
{
    Stop();
    if( index == 0 ) // High Level
    {
        //For high level do nothing
    }
    else // Low level
    {
        // For low level reset all control values
        ui->sliderMotor1->setValue(128);
        ui->sliderMotor2->setValue(128);
    }
}

void RobXControlDialog::Stop()
{
    mCtrl->Stop();
    
    ui->sliderMotor1->setSliderPosition(128);
    ui->sliderMotor2->setSliderPosition(128);
}

void RobXControlDialog::on_sliderMotor2_sliderReleased()
{
    mCtrl->Write(RobXControl::CMD_SET_SPEED_2, ui->sliderMotor2->value());
}

void RobXControlDialog::on_sliderMotor1_sliderReleased()
{
    mCtrl->Write(RobXControl::CMD_SET_SPEED_1, ui->sliderMotor1->value());
    if( ui->chkSyncMotors->isChecked() )
        mCtrl->Write(RobXControl::CMD_SET_SPEED_2, ui->sliderMotor1->value());
}

void RobXControlDialog::on_sldrMoveAmount_valueChanged(int value)
{
    ui->lblMoveAmount->setText(QString::number(value));
}

void RobXControlDialog::on_sldrMoveSpeed_valueChanged(int value)
{
    ui->lblMoveSpeed->setText(QString::number(value));
}

void RobXControlDialog::on_sldrTurnAmount_valueChanged(int value)
{
    ui->lblTurnAmount->setText(QString::number(value));
}

void RobXControlDialog::on_sldrTurnSpeed_valueChanged(int value)
{
    ui->lblTurnSpeed->setText(QString::number(value));
}

void RobXControlDialog::UpdateMotorInfo()
{
    try
    {
        //std::cout << "Update motor info ... \n" << std::flush;
        int v = mCtrl->GetVoltage();
        //std::cout << "1\n" << std::flush;
        float c1 = mCtrl->GetCurrent(1);
        //std::cout << "2\n" << std::flush;
        float c2 = mCtrl->GetCurrent(2);
        //std::cout << "3\n" << std::flush;
        int version = mCtrl->GetVersion();
        //std::cout << "4\n" << std::flush;
        int en1 = mCtrl->GetEncoderValue(1);
        //std::cout << "5\n" << std::flush;
        int en2 = mCtrl->GetEncoderValue(2);
        //std::cout << "6\n" << std::flush;
        
        ui->voltageLineEdit->setText(QString::number(v));
        ui->current1LineEdit->setText(QString::number(c1));
        ui->current2LineEdit->setText(QString::number(c2));
        ui->versionLineEdit->setText(QString::number(version));
        ui->encoder1LineEdit->setText(QString::number(en1));
        ui->encoder2LineEdit->setText(QString::number(en2));
    //TODO: Complete this.
    }
    catch(std::exception& ex)
    {
        std::cout << "(RobXControlDialog::UpdateMotorInfo) " << ex.what() << std::endl << std::flush;
    }
    catch(...)
    {
        std::cout << "(RobXControlDialog::UpdateMotorInfo) " << "Unknown exception occured.\n" << std::flush;
    }
}

void RobXControlDialog::SetController(RobXControl* ctrl)
{
    if( mCtrl )
        delete mCtrl;
    mCtrl = ctrl;
    ENABLE;
}


};
};
