/********************************************************************************
** Form generated from reading UI file 'CtrlDlg.ui'
**
** Created: Fri Jun 21 11:00:14 2013
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CTRLDLG_H
#define UI_CTRLDLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QFormLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

namespace KSRobot
{
namespace roboctrl
{

class Ui_RobXControlDlg
{
public:
    QTabWidget *tabControlLevel;
    QWidget *tabHighLevel;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_9;
    QGroupBox *grpMoveParamsStatic;
    QHBoxLayout *horizontalLayout_2;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *lblMoveAmountStatic;
    QLabel *lblMoveSpeedStatic;
    QVBoxLayout *verticalLayout_2;
    QSlider *sldrMoveAmount;
    QSlider *sldrMoveSpeed;
    QVBoxLayout *verticalLayout_3;
    QLabel *lblMoveAmount;
    QLabel *lblMoveSpeed;
    QVBoxLayout *verticalLayout_8;
    QPushButton *btnForward;
    QPushButton *btnBackward;
    QGroupBox *grpTurnParams;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_4;
    QLabel *lblTurnAmountStatic;
    QLabel *lblTurnSpeedStatic;
    QVBoxLayout *verticalLayout_5;
    QSlider *sldrTurnAmount;
    QSlider *sldrTurnSpeed;
    QVBoxLayout *verticalLayout_6;
    QLabel *lblTurnAmount;
    QLabel *lblTurnSpeed;
    QVBoxLayout *verticalLayout_10;
    QPushButton *btnTurnRight;
    QPushButton *btnTurnLeft;
    QPushButton *btnStop;
    QWidget *tabLowLevel;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_11;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_13;
    QSlider *sliderMotor1;
    QSlider *sliderMotor2;
    QVBoxLayout *verticalLayout_14;
    QLabel *lblMotor1;
    QLabel *lblMotor2;
    QCheckBox *chkSyncMotors;
    QHBoxLayout *horizontalLayout_6;
    QFormLayout *formLayout;
    QLabel *encoder1Label;
    QLabel *encoder2Label;
    QLineEdit *encoder2LineEdit;
    QLabel *voltageLabel;
    QLineEdit *voltageLineEdit;
    QLabel *current1Label;
    QLineEdit *current1LineEdit;
    QLabel *current2Label;
    QLineEdit *current2LineEdit;
    QLabel *versionLabel;
    QLineEdit *versionLineEdit;
    QLineEdit *encoder1LineEdit;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnStop2;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout_2;
    QLineEdit *deviceLineEdit;
    QLabel *deviceLabel;
    QPushButton *btnConnectDevice;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(596, 413);
        tabControlLevel = new QTabWidget(Dialog);
        tabControlLevel->setObjectName(QString::fromUtf8("tabControlLevel"));
        tabControlLevel->setGeometry(QRect(10, 10, 571, 331));
        tabHighLevel = new QWidget();
        tabHighLevel->setObjectName(QString::fromUtf8("tabHighLevel"));
        verticalLayoutWidget_6 = new QWidget(tabHighLevel);
        verticalLayoutWidget_6->setObjectName(QString::fromUtf8("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(0, 0, 561, 291));
        verticalLayout_9 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        grpMoveParamsStatic = new QGroupBox(verticalLayoutWidget_6);
        grpMoveParamsStatic->setObjectName(QString::fromUtf8("grpMoveParamsStatic"));
        horizontalLayout_2 = new QHBoxLayout(grpMoveParamsStatic);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        lblMoveAmountStatic = new QLabel(grpMoveParamsStatic);
        lblMoveAmountStatic->setObjectName(QString::fromUtf8("lblMoveAmountStatic"));

        verticalLayout->addWidget(lblMoveAmountStatic);

        lblMoveSpeedStatic = new QLabel(grpMoveParamsStatic);
        lblMoveSpeedStatic->setObjectName(QString::fromUtf8("lblMoveSpeedStatic"));

        verticalLayout->addWidget(lblMoveSpeedStatic);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetFixedSize);
        sldrMoveAmount = new QSlider(grpMoveParamsStatic);
        sldrMoveAmount->setObjectName(QString::fromUtf8("sldrMoveAmount"));
        sldrMoveAmount->setMaximum(300);
        sldrMoveAmount->setSliderPosition(0);
        sldrMoveAmount->setTracking(true);
        sldrMoveAmount->setOrientation(Qt::Horizontal);
        sldrMoveAmount->setInvertedAppearance(false);
        sldrMoveAmount->setInvertedControls(false);

        verticalLayout_2->addWidget(sldrMoveAmount);

        sldrMoveSpeed = new QSlider(grpMoveParamsStatic);
        sldrMoveSpeed->setObjectName(QString::fromUtf8("sldrMoveSpeed"));
        sldrMoveSpeed->setMaximum(12);
        sldrMoveSpeed->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(sldrMoveSpeed);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setSizeConstraint(QLayout::SetFixedSize);
        verticalLayout_3->setContentsMargins(-1, -1, 20, -1);
        lblMoveAmount = new QLabel(grpMoveParamsStatic);
        lblMoveAmount->setObjectName(QString::fromUtf8("lblMoveAmount"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(lblMoveAmount->sizePolicy().hasHeightForWidth());
        lblMoveAmount->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(lblMoveAmount);

        lblMoveSpeed = new QLabel(grpMoveParamsStatic);
        lblMoveSpeed->setObjectName(QString::fromUtf8("lblMoveSpeed"));

        verticalLayout_3->addWidget(lblMoveSpeed);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        btnForward = new QPushButton(grpMoveParamsStatic);
        btnForward->setObjectName(QString::fromUtf8("btnForward"));

        verticalLayout_8->addWidget(btnForward);

        btnBackward = new QPushButton(grpMoveParamsStatic);
        btnBackward->setObjectName(QString::fromUtf8("btnBackward"));

        verticalLayout_8->addWidget(btnBackward);


        horizontalLayout->addLayout(verticalLayout_8);


        horizontalLayout_2->addLayout(horizontalLayout);


        verticalLayout_9->addWidget(grpMoveParamsStatic);

        grpTurnParams = new QGroupBox(verticalLayoutWidget_6);
        grpTurnParams->setObjectName(QString::fromUtf8("grpTurnParams"));
        horizontalLayout_3 = new QHBoxLayout(grpTurnParams);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        lblTurnAmountStatic = new QLabel(grpTurnParams);
        lblTurnAmountStatic->setObjectName(QString::fromUtf8("lblTurnAmountStatic"));

        verticalLayout_4->addWidget(lblTurnAmountStatic);

        lblTurnSpeedStatic = new QLabel(grpTurnParams);
        lblTurnSpeedStatic->setObjectName(QString::fromUtf8("lblTurnSpeedStatic"));

        verticalLayout_4->addWidget(lblTurnSpeedStatic);


        horizontalLayout_4->addLayout(verticalLayout_4);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        sldrTurnAmount = new QSlider(grpTurnParams);
        sldrTurnAmount->setObjectName(QString::fromUtf8("sldrTurnAmount"));
        sldrTurnAmount->setMaximum(90);
        sldrTurnAmount->setOrientation(Qt::Horizontal);

        verticalLayout_5->addWidget(sldrTurnAmount);

        sldrTurnSpeed = new QSlider(grpTurnParams);
        sldrTurnSpeed->setObjectName(QString::fromUtf8("sldrTurnSpeed"));
        sldrTurnSpeed->setOrientation(Qt::Horizontal);

        verticalLayout_5->addWidget(sldrTurnSpeed);


        horizontalLayout_4->addLayout(verticalLayout_5);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setSizeConstraint(QLayout::SetFixedSize);
        verticalLayout_6->setContentsMargins(-1, -1, 20, -1);
        lblTurnAmount = new QLabel(grpTurnParams);
        lblTurnAmount->setObjectName(QString::fromUtf8("lblTurnAmount"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lblTurnAmount->sizePolicy().hasHeightForWidth());
        lblTurnAmount->setSizePolicy(sizePolicy1);

        verticalLayout_6->addWidget(lblTurnAmount);

        lblTurnSpeed = new QLabel(grpTurnParams);
        lblTurnSpeed->setObjectName(QString::fromUtf8("lblTurnSpeed"));

        verticalLayout_6->addWidget(lblTurnSpeed);


        horizontalLayout_4->addLayout(verticalLayout_6);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        btnTurnRight = new QPushButton(grpTurnParams);
        btnTurnRight->setObjectName(QString::fromUtf8("btnTurnRight"));

        verticalLayout_10->addWidget(btnTurnRight);

        btnTurnLeft = new QPushButton(grpTurnParams);
        btnTurnLeft->setObjectName(QString::fromUtf8("btnTurnLeft"));

        verticalLayout_10->addWidget(btnTurnLeft);


        horizontalLayout_4->addLayout(verticalLayout_10);


        horizontalLayout_3->addLayout(horizontalLayout_4);


        verticalLayout_9->addWidget(grpTurnParams);

        btnStop = new QPushButton(verticalLayoutWidget_6);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));

        verticalLayout_9->addWidget(btnStop);

        tabControlLevel->addTab(tabHighLevel, QString());
        tabLowLevel = new QWidget();
        tabLowLevel->setObjectName(QString::fromUtf8("tabLowLevel"));
        verticalLayoutWidget_3 = new QWidget(tabLowLevel);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(0, 0, 561, 291));
        verticalLayout_11 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        verticalLayout_11->setContentsMargins(0, 0, 0, 0);
        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        verticalLayout_13->setSizeConstraint(QLayout::SetFixedSize);
        sliderMotor1 = new QSlider(verticalLayoutWidget_3);
        sliderMotor1->setObjectName(QString::fromUtf8("sliderMotor1"));
        sliderMotor1->setMaximum(255);
        sliderMotor1->setValue(128);
        sliderMotor1->setOrientation(Qt::Horizontal);

        verticalLayout_13->addWidget(sliderMotor1);

        sliderMotor2 = new QSlider(verticalLayoutWidget_3);
        sliderMotor2->setObjectName(QString::fromUtf8("sliderMotor2"));
        sliderMotor2->setMaximum(255);
        sliderMotor2->setValue(128);
        sliderMotor2->setOrientation(Qt::Horizontal);

        verticalLayout_13->addWidget(sliderMotor2);


        horizontalLayout_5->addLayout(verticalLayout_13);

        verticalLayout_14 = new QVBoxLayout();
        verticalLayout_14->setSpacing(6);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        verticalLayout_14->setSizeConstraint(QLayout::SetFixedSize);
        lblMotor1 = new QLabel(verticalLayoutWidget_3);
        lblMotor1->setObjectName(QString::fromUtf8("lblMotor1"));

        verticalLayout_14->addWidget(lblMotor1);

        lblMotor2 = new QLabel(verticalLayoutWidget_3);
        lblMotor2->setObjectName(QString::fromUtf8("lblMotor2"));

        verticalLayout_14->addWidget(lblMotor2);


        horizontalLayout_5->addLayout(verticalLayout_14);


        verticalLayout_12->addLayout(horizontalLayout_5);

        chkSyncMotors = new QCheckBox(verticalLayoutWidget_3);
        chkSyncMotors->setObjectName(QString::fromUtf8("chkSyncMotors"));

        verticalLayout_12->addWidget(chkSyncMotors);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        encoder1Label = new QLabel(verticalLayoutWidget_3);
        encoder1Label->setObjectName(QString::fromUtf8("encoder1Label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, encoder1Label);

        encoder2Label = new QLabel(verticalLayoutWidget_3);
        encoder2Label->setObjectName(QString::fromUtf8("encoder2Label"));

        formLayout->setWidget(1, QFormLayout::LabelRole, encoder2Label);

        encoder2LineEdit = new QLineEdit(verticalLayoutWidget_3);
        encoder2LineEdit->setObjectName(QString::fromUtf8("encoder2LineEdit"));
        encoder2LineEdit->setReadOnly(true);

        formLayout->setWidget(1, QFormLayout::FieldRole, encoder2LineEdit);

        voltageLabel = new QLabel(verticalLayoutWidget_3);
        voltageLabel->setObjectName(QString::fromUtf8("voltageLabel"));

        formLayout->setWidget(2, QFormLayout::LabelRole, voltageLabel);

        voltageLineEdit = new QLineEdit(verticalLayoutWidget_3);
        voltageLineEdit->setObjectName(QString::fromUtf8("voltageLineEdit"));
        voltageLineEdit->setReadOnly(true);

        formLayout->setWidget(2, QFormLayout::FieldRole, voltageLineEdit);

        current1Label = new QLabel(verticalLayoutWidget_3);
        current1Label->setObjectName(QString::fromUtf8("current1Label"));

        formLayout->setWidget(3, QFormLayout::LabelRole, current1Label);

        current1LineEdit = new QLineEdit(verticalLayoutWidget_3);
        current1LineEdit->setObjectName(QString::fromUtf8("current1LineEdit"));
        current1LineEdit->setReadOnly(true);

        formLayout->setWidget(3, QFormLayout::FieldRole, current1LineEdit);

        current2Label = new QLabel(verticalLayoutWidget_3);
        current2Label->setObjectName(QString::fromUtf8("current2Label"));

        formLayout->setWidget(4, QFormLayout::LabelRole, current2Label);

        current2LineEdit = new QLineEdit(verticalLayoutWidget_3);
        current2LineEdit->setObjectName(QString::fromUtf8("current2LineEdit"));
        current2LineEdit->setReadOnly(true);

        formLayout->setWidget(4, QFormLayout::FieldRole, current2LineEdit);

        versionLabel = new QLabel(verticalLayoutWidget_3);
        versionLabel->setObjectName(QString::fromUtf8("versionLabel"));

        formLayout->setWidget(5, QFormLayout::LabelRole, versionLabel);

        versionLineEdit = new QLineEdit(verticalLayoutWidget_3);
        versionLineEdit->setObjectName(QString::fromUtf8("versionLineEdit"));
        versionLineEdit->setReadOnly(true);

        formLayout->setWidget(5, QFormLayout::FieldRole, versionLineEdit);

        encoder1LineEdit = new QLineEdit(verticalLayoutWidget_3);
        encoder1LineEdit->setObjectName(QString::fromUtf8("encoder1LineEdit"));
        encoder1LineEdit->setReadOnly(true);

        formLayout->setWidget(0, QFormLayout::FieldRole, encoder1LineEdit);


        horizontalLayout_6->addLayout(formLayout);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer);


        verticalLayout_12->addLayout(horizontalLayout_6);

        btnStop2 = new QPushButton(verticalLayoutWidget_3);
        btnStop2->setObjectName(QString::fromUtf8("btnStop2"));

        verticalLayout_12->addWidget(btnStop2);


        verticalLayout_11->addLayout(verticalLayout_12);

        tabControlLevel->addTab(tabLowLevel, QString());
        formLayoutWidget = new QWidget(Dialog);
        formLayoutWidget->setObjectName(QString::fromUtf8("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(10, 380, 241, 31));
        formLayout_2 = new QFormLayout(formLayoutWidget);
        formLayout_2->setSpacing(6);
        formLayout_2->setContentsMargins(11, 11, 11, 11);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        formLayout_2->setContentsMargins(0, 0, 0, 0);
        deviceLineEdit = new QLineEdit(formLayoutWidget);
        deviceLineEdit->setObjectName(QString::fromUtf8("deviceLineEdit"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, deviceLineEdit);

        deviceLabel = new QLabel(formLayoutWidget);
        deviceLabel->setObjectName(QString::fromUtf8("deviceLabel"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, deviceLabel);

        btnConnectDevice = new QPushButton(Dialog);
        btnConnectDevice->setObjectName(QString::fromUtf8("btnConnectDevice"));
        btnConnectDevice->setGeometry(QRect(270, 380, 81, 23));

        retranslateUi(Dialog);

        tabControlLevel->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0, QApplication::UnicodeUTF8));
        grpMoveParamsStatic->setTitle(QApplication::translate("Dialog", "Move Parameters", 0, QApplication::UnicodeUTF8));
        lblMoveAmountStatic->setText(QApplication::translate("Dialog", "Amount:", 0, QApplication::UnicodeUTF8));
        lblMoveSpeedStatic->setText(QApplication::translate("Dialog", "Speed:", 0, QApplication::UnicodeUTF8));
        lblMoveAmount->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        lblMoveSpeed->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        btnForward->setText(QApplication::translate("Dialog", "Forward", 0, QApplication::UnicodeUTF8));
        btnBackward->setText(QApplication::translate("Dialog", "Backward", 0, QApplication::UnicodeUTF8));
        grpTurnParams->setTitle(QApplication::translate("Dialog", "Turn Parameters", 0, QApplication::UnicodeUTF8));
        lblTurnAmountStatic->setText(QApplication::translate("Dialog", "Amount:", 0, QApplication::UnicodeUTF8));
        lblTurnSpeedStatic->setText(QApplication::translate("Dialog", "Speed:", 0, QApplication::UnicodeUTF8));
        lblTurnAmount->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        lblTurnSpeed->setText(QApplication::translate("Dialog", "0", 0, QApplication::UnicodeUTF8));
        btnTurnRight->setText(QApplication::translate("Dialog", "Turn Right", 0, QApplication::UnicodeUTF8));
        btnTurnLeft->setText(QApplication::translate("Dialog", "Turn Left", 0, QApplication::UnicodeUTF8));
        btnStop->setText(QApplication::translate("Dialog", "STOP", 0, QApplication::UnicodeUTF8));
        tabControlLevel->setTabText(tabControlLevel->indexOf(tabHighLevel), QApplication::translate("Dialog", "High Level", 0, QApplication::UnicodeUTF8));
        lblMotor1->setText(QApplication::translate("Dialog", "128", 0, QApplication::UnicodeUTF8));
        lblMotor2->setText(QApplication::translate("Dialog", "128", 0, QApplication::UnicodeUTF8));
        chkSyncMotors->setText(QApplication::translate("Dialog", "Sync Motors", 0, QApplication::UnicodeUTF8));
        encoder1Label->setText(QApplication::translate("Dialog", "Encoder 1:", 0, QApplication::UnicodeUTF8));
        encoder2Label->setText(QApplication::translate("Dialog", "Encoder 2:", 0, QApplication::UnicodeUTF8));
        voltageLabel->setText(QApplication::translate("Dialog", "Voltage:", 0, QApplication::UnicodeUTF8));
        current1Label->setText(QApplication::translate("Dialog", "Current 1:", 0, QApplication::UnicodeUTF8));
        current2Label->setText(QApplication::translate("Dialog", "Current 2", 0, QApplication::UnicodeUTF8));
        versionLabel->setText(QApplication::translate("Dialog", "Version:", 0, QApplication::UnicodeUTF8));
        btnStop2->setText(QApplication::translate("Dialog", "STOP", 0, QApplication::UnicodeUTF8));
        tabControlLevel->setTabText(tabControlLevel->indexOf(tabLowLevel), QApplication::translate("Dialog", "Low Level", 0, QApplication::UnicodeUTF8));
        deviceLabel->setText(QApplication::translate("Dialog", "Device:", 0, QApplication::UnicodeUTF8));
        btnConnectDevice->setText(QApplication::translate("Dialog", "Connect", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class RobXCtrlDlg: public Ui_RobXControlDlg {};
} // namespace Ui

};
};

#endif // UI_CTRLDLG_H
