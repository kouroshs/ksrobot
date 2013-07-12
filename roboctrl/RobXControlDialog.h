#ifndef CTRLDLG_H
#define CTRLDLG_H

#include <QDialog>
#include <roboctrl/RobXControl.h>

class QTimer;

namespace KSRobot
{
namespace roboctrl
{

namespace Ui {
class RobXCtrlDlg;
}

class RobXControlDialog : public QDialog
{
    Q_OBJECT
public:
    explicit RobXControlDialog(QWidget *parent = 0);
    ~RobXControlDialog();


    void                SetController(RobXControl* ctrl);
    
private slots:
    void on_btnConnectDevice_clicked();
    void on_btnStop_clicked();
    void on_btnForward_clicked();
    void on_btnBackward_clicked();
    void on_btnTurnRight_clicked();
    void on_btnTurnLeft_clicked();
    void on_btnStop2_clicked();
    void on_chkSyncMotors_clicked(bool checked);
    void on_sliderMotor1_valueChanged(int value);
    void on_sliderMotor2_valueChanged(int value);
    void on_tabControlLevel_currentChanged(int index);
    void on_sliderMotor2_sliderReleased();
    void on_sliderMotor1_sliderReleased();
    void on_sldrMoveAmount_valueChanged(int value);
    void on_sldrMoveSpeed_valueChanged(int value);
    void on_sldrTurnAmount_valueChanged(int value);
    void on_sldrTurnSpeed_valueChanged(int value);

    void UpdateMotorInfo();
private:
    void Disconnect();
    void Connect(const QString& device);
    void Stop();
private:
    Ui::RobXCtrlDlg *ui;
    RobXControl*                mCtrl;
    QTimer*                     mEncoderUpdateTimer;
};

};
};

#endif // CTRLDLG_H
