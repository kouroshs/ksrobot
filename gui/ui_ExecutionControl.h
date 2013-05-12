#ifndef UI_EXECUTIONCONTROL_H
#define UI_EXECUTIONCONTROL_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

namespace KSRobot
{
namespace gui
{

class Ui_ExecutionControl
{
public:
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QTabWidget *mTabSettings;
    QWidget *mTabKinect;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *mTabKinectLayout;
    QGroupBox *mGrpKinectSource;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *mLayoutKinectSourceGrp;
    QRadioButton *mRdBtnKinectDevice;
    QRadioButton *mRdBtnKinectFile;
    QSpacerItem *verticalSpacer_4;
    QHBoxLayout *mLayoutHorizKinect;
    QLineEdit *mKinectDevPath;
    QPushButton *mBtnBrowseKinect;
    QWidget *mTabFovis;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *mLayoutFovis;
    QCheckBox *mChkEnableFovis;
    QSpacerItem *verticalSpacer_2;
    QWidget *mTabiSAM;
    QVBoxLayout *verticalLayout_7;
    QVBoxLayout *mLayoutSAM;
    QCheckBox *mChkEnableSAM;
    QSpacerItem *verticalSpacer;
    QWidget *mTabOctoMap;
    QVBoxLayout *verticalLayout_8;
    QVBoxLayout *mLayoutOctoMap;
    QCheckBox *mChkEnableOctomap;
    QSpacerItem *verticalSpacer_5;
    QWidget *mTabOMPL;
    QVBoxLayout *verticalLayout_9;
    QVBoxLayout *mLayoutOMPL;
    QCheckBox *mChkEnableOMPL;
    QSpacerItem *verticalSpacer_6;
    QWidget *mTabComm;
    QVBoxLayout *verticalLayout_10;
    QVBoxLayout *mLayoutComm;
    QCheckBox *mChkEnableComm;
    QHBoxLayout *horizontalLayout;
    QLabel *nLblHostName;
    QLineEdit *mEditHost;
    QLabel *mLblPortNumber;
    QLineEdit *mEditPort;
    QSpacerItem *verticalSpacer_7;
    QWidget *mTabGUI;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *mLayoutGUI;
    QCheckBox *mChkViewRGBD;
    QCheckBox *mChkViewPointCloud;
    QCheckBox *mChkViewFovisResults;
    QCheckBox *mChkViewOctomap;
    QCheckBox *mChkViewOMPL;
    QSpacerItem *verticalSpacer_3;
    QHBoxLayout *mLayoutH2;
    QPushButton *mBtnStart;
    QPushButton *mBtnStop;
    QSpacerItem *mSpacerH2;

    void setupUi(QWidget *ExecutionControl)
    {
        if (ExecutionControl->objectName().isEmpty())
            ExecutionControl->setObjectName(QString::fromUtf8("ExecutionControl"));
        ExecutionControl->resize(467, 237);
        verticalLayout_3 = new QVBoxLayout(ExecutionControl);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        mTabSettings = new QTabWidget(ExecutionControl);
        mTabSettings->setObjectName(QString::fromUtf8("mTabSettings"));
        mTabKinect = new QWidget();
        mTabKinect->setObjectName(QString::fromUtf8("mTabKinect"));
        verticalLayout = new QVBoxLayout(mTabKinect);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        mTabKinectLayout = new QVBoxLayout();
        mTabKinectLayout->setObjectName(QString::fromUtf8("mTabKinectLayout"));
        mGrpKinectSource = new QGroupBox(mTabKinect);
        mGrpKinectSource->setObjectName(QString::fromUtf8("mGrpKinectSource"));
        verticalLayout_4 = new QVBoxLayout(mGrpKinectSource);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        mLayoutKinectSourceGrp = new QVBoxLayout();
        mLayoutKinectSourceGrp->setObjectName(QString::fromUtf8("mLayoutKinectSourceGrp"));
        mRdBtnKinectDevice = new QRadioButton(mGrpKinectSource);
        mRdBtnKinectDevice->setObjectName(QString::fromUtf8("mRdBtnKinectDevice"));

        mLayoutKinectSourceGrp->addWidget(mRdBtnKinectDevice);

        mRdBtnKinectFile = new QRadioButton(mGrpKinectSource);
        mRdBtnKinectFile->setObjectName(QString::fromUtf8("mRdBtnKinectFile"));

        mLayoutKinectSourceGrp->addWidget(mRdBtnKinectFile);


        verticalLayout_4->addLayout(mLayoutKinectSourceGrp);


        mTabKinectLayout->addWidget(mGrpKinectSource);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mTabKinectLayout->addItem(verticalSpacer_4);

        mLayoutHorizKinect = new QHBoxLayout();
        mLayoutHorizKinect->setObjectName(QString::fromUtf8("mLayoutHorizKinect"));
        mKinectDevPath = new QLineEdit(mTabKinect);
        mKinectDevPath->setObjectName(QString::fromUtf8("mKinectDevPath"));

        mLayoutHorizKinect->addWidget(mKinectDevPath);

        mBtnBrowseKinect = new QPushButton(mTabKinect);
        mBtnBrowseKinect->setObjectName(QString::fromUtf8("mBtnBrowseKinect"));

        mLayoutHorizKinect->addWidget(mBtnBrowseKinect);


        mTabKinectLayout->addLayout(mLayoutHorizKinect);


        verticalLayout->addLayout(mTabKinectLayout);

        mTabSettings->addTab(mTabKinect, QString());
        mTabFovis = new QWidget();
        mTabFovis->setObjectName(QString::fromUtf8("mTabFovis"));
        verticalLayout_5 = new QVBoxLayout(mTabFovis);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        mLayoutFovis = new QVBoxLayout();
        mLayoutFovis->setObjectName(QString::fromUtf8("mLayoutFovis"));
        mChkEnableFovis = new QCheckBox(mTabFovis);
        mChkEnableFovis->setObjectName(QString::fromUtf8("mChkEnableFovis"));
        mChkEnableFovis->setChecked(false);

        mLayoutFovis->addWidget(mChkEnableFovis);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutFovis->addItem(verticalSpacer_2);


        verticalLayout_5->addLayout(mLayoutFovis);

        mTabSettings->addTab(mTabFovis, QString());
        mTabiSAM = new QWidget();
        mTabiSAM->setObjectName(QString::fromUtf8("mTabiSAM"));
        verticalLayout_7 = new QVBoxLayout(mTabiSAM);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        mLayoutSAM = new QVBoxLayout();
        mLayoutSAM->setObjectName(QString::fromUtf8("mLayoutSAM"));
        mChkEnableSAM = new QCheckBox(mTabiSAM);
        mChkEnableSAM->setObjectName(QString::fromUtf8("mChkEnableSAM"));

        mLayoutSAM->addWidget(mChkEnableSAM);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutSAM->addItem(verticalSpacer);


        verticalLayout_7->addLayout(mLayoutSAM);

        mTabSettings->addTab(mTabiSAM, QString());
        mTabOctoMap = new QWidget();
        mTabOctoMap->setObjectName(QString::fromUtf8("mTabOctoMap"));
        verticalLayout_8 = new QVBoxLayout(mTabOctoMap);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        mLayoutOctoMap = new QVBoxLayout();
        mLayoutOctoMap->setObjectName(QString::fromUtf8("mLayoutOctoMap"));
        mChkEnableOctomap = new QCheckBox(mTabOctoMap);
        mChkEnableOctomap->setObjectName(QString::fromUtf8("mChkEnableOctomap"));

        mLayoutOctoMap->addWidget(mChkEnableOctomap);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutOctoMap->addItem(verticalSpacer_5);


        verticalLayout_8->addLayout(mLayoutOctoMap);

        mTabSettings->addTab(mTabOctoMap, QString());
        mTabOMPL = new QWidget();
        mTabOMPL->setObjectName(QString::fromUtf8("mTabOMPL"));
        verticalLayout_9 = new QVBoxLayout(mTabOMPL);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        mLayoutOMPL = new QVBoxLayout();
        mLayoutOMPL->setObjectName(QString::fromUtf8("mLayoutOMPL"));
        mChkEnableOMPL = new QCheckBox(mTabOMPL);
        mChkEnableOMPL->setObjectName(QString::fromUtf8("mChkEnableOMPL"));

        mLayoutOMPL->addWidget(mChkEnableOMPL);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutOMPL->addItem(verticalSpacer_6);


        verticalLayout_9->addLayout(mLayoutOMPL);

        mTabSettings->addTab(mTabOMPL, QString());
        mTabComm = new QWidget();
        mTabComm->setObjectName(QString::fromUtf8("mTabComm"));
        verticalLayout_10 = new QVBoxLayout(mTabComm);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        mLayoutComm = new QVBoxLayout();
        mLayoutComm->setObjectName(QString::fromUtf8("mLayoutComm"));
        mChkEnableComm = new QCheckBox(mTabComm);
        mChkEnableComm->setObjectName(QString::fromUtf8("mChkEnableComm"));

        mLayoutComm->addWidget(mChkEnableComm);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        nLblHostName = new QLabel(mTabComm);
        nLblHostName->setObjectName(QString::fromUtf8("nLblHostName"));

        horizontalLayout->addWidget(nLblHostName);

        mEditHost = new QLineEdit(mTabComm);
        mEditHost->setObjectName(QString::fromUtf8("mEditHost"));
        mEditHost->setMinimumSize(QSize(230, 23));

        horizontalLayout->addWidget(mEditHost);

        mLblPortNumber = new QLabel(mTabComm);
        mLblPortNumber->setObjectName(QString::fromUtf8("mLblPortNumber"));

        horizontalLayout->addWidget(mLblPortNumber);

        mEditPort = new QLineEdit(mTabComm);
        mEditPort->setObjectName(QString::fromUtf8("mEditPort"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mEditPort->sizePolicy().hasHeightForWidth());
        mEditPort->setSizePolicy(sizePolicy);
        mEditPort->setMinimumSize(QSize(75, 23));
        mEditPort->setMaximumSize(QSize(75, 23));
        mEditPort->setBaseSize(QSize(100, 0));
        mEditPort->setMaxLength(5);

        horizontalLayout->addWidget(mEditPort);

        horizontalLayout->setStretch(1, 1);

        mLayoutComm->addLayout(horizontalLayout);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutComm->addItem(verticalSpacer_7);


        verticalLayout_10->addLayout(mLayoutComm);

        mTabSettings->addTab(mTabComm, QString());
        mTabGUI = new QWidget();
        mTabGUI->setObjectName(QString::fromUtf8("mTabGUI"));
        verticalLayout_6 = new QVBoxLayout(mTabGUI);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        mLayoutGUI = new QVBoxLayout();
        mLayoutGUI->setObjectName(QString::fromUtf8("mLayoutGUI"));
        mChkViewRGBD = new QCheckBox(mTabGUI);
        mChkViewRGBD->setObjectName(QString::fromUtf8("mChkViewRGBD"));

        mLayoutGUI->addWidget(mChkViewRGBD);

        mChkViewPointCloud = new QCheckBox(mTabGUI);
        mChkViewPointCloud->setObjectName(QString::fromUtf8("mChkViewPointCloud"));

        mLayoutGUI->addWidget(mChkViewPointCloud);

        mChkViewFovisResults = new QCheckBox(mTabGUI);
        mChkViewFovisResults->setObjectName(QString::fromUtf8("mChkViewFovisResults"));

        mLayoutGUI->addWidget(mChkViewFovisResults);

        mChkViewOctomap = new QCheckBox(mTabGUI);
        mChkViewOctomap->setObjectName(QString::fromUtf8("mChkViewOctomap"));

        mLayoutGUI->addWidget(mChkViewOctomap);

        mChkViewOMPL = new QCheckBox(mTabGUI);
        mChkViewOMPL->setObjectName(QString::fromUtf8("mChkViewOMPL"));

        mLayoutGUI->addWidget(mChkViewOMPL);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        mLayoutGUI->addItem(verticalSpacer_3);


        verticalLayout_6->addLayout(mLayoutGUI);

        mTabSettings->addTab(mTabGUI, QString());

        verticalLayout_2->addWidget(mTabSettings);

        mLayoutH2 = new QHBoxLayout();
        mLayoutH2->setObjectName(QString::fromUtf8("mLayoutH2"));
        mBtnStart = new QPushButton(ExecutionControl);
        mBtnStart->setObjectName(QString::fromUtf8("mBtnStart"));

        mLayoutH2->addWidget(mBtnStart);

        mBtnStop = new QPushButton(ExecutionControl);
        mBtnStop->setObjectName(QString::fromUtf8("mBtnStop"));
        mBtnStop->setEnabled(false);

        mLayoutH2->addWidget(mBtnStop);

        mSpacerH2 = new QSpacerItem(100, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        mLayoutH2->addItem(mSpacerH2);


        verticalLayout_2->addLayout(mLayoutH2);


        verticalLayout_3->addLayout(verticalLayout_2);


        retranslateUi(ExecutionControl);

        mTabSettings->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(ExecutionControl);
    } // setupUi

    void retranslateUi(QWidget *ExecutionControl)
    {
        ExecutionControl->setWindowTitle(QApplication::translate("ExecutionControl", "Form", 0, QApplication::UnicodeUTF8));
        mGrpKinectSource->setTitle(QApplication::translate("ExecutionControl", "Kinect Source", 0, QApplication::UnicodeUTF8));
        mRdBtnKinectDevice->setText(QApplication::translate("ExecutionControl", "Kinect Device", 0, QApplication::UnicodeUTF8));
        mRdBtnKinectFile->setText(QApplication::translate("ExecutionControl", "Dataset", 0, QApplication::UnicodeUTF8));
        mBtnBrowseKinect->setText(QApplication::translate("ExecutionControl", "Browse", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabKinect), QApplication::translate("ExecutionControl", "Kinect", 0, QApplication::UnicodeUTF8));
        mChkEnableFovis->setText(QApplication::translate("ExecutionControl", "Fovis Enabled", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabFovis), QApplication::translate("ExecutionControl", "Fovis", 0, QApplication::UnicodeUTF8));
        mChkEnableSAM->setText(QApplication::translate("ExecutionControl", "Enable iSAM", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabiSAM), QApplication::translate("ExecutionControl", "iSAM", 0, QApplication::UnicodeUTF8));
        mChkEnableOctomap->setText(QApplication::translate("ExecutionControl", "Enable OctoMap", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabOctoMap), QApplication::translate("ExecutionControl", "OctoMap", 0, QApplication::UnicodeUTF8));
        mChkEnableOMPL->setText(QApplication::translate("ExecutionControl", "Enable OMPL", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabOMPL), QApplication::translate("ExecutionControl", "OMPL", 0, QApplication::UnicodeUTF8));
        mChkEnableComm->setText(QApplication::translate("ExecutionControl", "Enable Communications", 0, QApplication::UnicodeUTF8));
        nLblHostName->setText(QApplication::translate("ExecutionControl", "Host:", 0, QApplication::UnicodeUTF8));
        mLblPortNumber->setText(QApplication::translate("ExecutionControl", "Port:", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabComm), QApplication::translate("ExecutionControl", "Comm", 0, QApplication::UnicodeUTF8));
        mChkViewRGBD->setText(QApplication::translate("ExecutionControl", "View RGBD Images", 0, QApplication::UnicodeUTF8));
        mChkViewPointCloud->setText(QApplication::translate("ExecutionControl", "View PointClouds", 0, QApplication::UnicodeUTF8));
        mChkViewFovisResults->setText(QApplication::translate("ExecutionControl", "View Fovis Results", 0, QApplication::UnicodeUTF8));
        mChkViewOctomap->setText(QApplication::translate("ExecutionControl", "View Octomap Map", 0, QApplication::UnicodeUTF8));
        mChkViewOMPL->setText(QApplication::translate("ExecutionControl", "View OMPL Results", 0, QApplication::UnicodeUTF8));
        mTabSettings->setTabText(mTabSettings->indexOf(mTabGUI), QApplication::translate("ExecutionControl", "GUI", 0, QApplication::UnicodeUTF8));
        mBtnStart->setText(QApplication::translate("ExecutionControl", "Start", 0, QApplication::UnicodeUTF8));
        mBtnStop->setText(QApplication::translate("ExecutionControl", "Stop", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ExecutionControl: public Ui_ExecutionControl {};
} // namespace Ui

}
}

#endif // UI_EXECUTIONCONTROL_H
