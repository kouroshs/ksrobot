/**
 * @file   MainWindow.cpp
 * @Author Kourosh Sartipi (kourosh.sartipi@gmail.com)
 * @date   March, 2013
 * @brief  Source file of the main window of GUI.
 *
 */


#include <gui/MainWindow.h>
#include <gui/Utils.h>
#include <boost/bimap.hpp>

#include <iostream>

#include <gui/ImageContainer.h>
#include <gui/LogContainer.h>
#include <gui/ExecutionControl.h>
#include <gui/StatisticsContainer.h>
#include <gui/PointCloudViewer.h>
#include <gui/Utils.h>
#include <gui/LogicBridge.h>


#include <QAction>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QMenu>
#include <QMenuBar>
#include <QEvent>
#include <QLabel>
#include <QToolBar>
#include <QDebug>
#include <QMessageBox>
#include <QMetaObject>
#include <QResizeEvent>
#include <QSignalMapper>

using std::cout;
using std::flush;

using namespace KSRobot::gui;
using namespace KSRobot::common;

QVector<MainWindow*>  MainWindow::s_windowLogReceiverList;
bool MainWindow::s_bHandlerInitialized = false;
QtMsgHandler MainWindow::s_prevMsgHandler;

MainWindow::MainWindow() : QMainWindow()
{
}

MainWindow::~MainWindow()
{
    Cleanup();
}

void MainWindow::InitWindow(ProgramOptions::Ptr root, ProgramOptions::Ptr poGUI)
{
    mRootPO = root;
    mGuiPO = poGUI;
    
    setWindowTitle(tr("KSRobot"));
    
    if( objectName() == "" )
        this->setObjectName("MainWindow");
    
    mMdiArea = new QMdiArea(this);
    SET_QTOBJECTNAME(mMdiArea);
    mMdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mMdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setCentralWidget(mMdiArea);
    
    CreateActions();
    CreateMenus();
    CreateToolbars();
    CreateStatusbar();
    SetupUI();

    SetupLogic();
    
    showMaximized();
}

void MainWindow::SetupLogic()
{
    mLogic = new LogicBridge(this);
    SET_QTOBJECTNAME(mLogic);
    
    mLogic->SetRGBDSkip(5);
    mLogic->SetPointCloudSkip(5);
    
    connect(mExecControl, SIGNAL(OnStart(common::ExecCtrlData)), mLogic, SLOT(OnStart(common::ExecCtrlData)));
    connect(mExecControl, SIGNAL(OnStop()), mLogic, SLOT(OnStop()));
    connect(mLogic, SIGNAL(OnError(QString)), this, SLOT(OnLogicError(QString)), Qt::QueuedConnection);
    connect(mLogic, SIGNAL(OnPointCloud(common::KinectPointCloud::ConstPtr)), 
            mPCViewer, SLOT(UpdataPointCloud(common::KinectPointCloud::ConstPtr)), Qt::QueuedConnection);
    connect(mLogic, SIGNAL(OnRGBD(QImage,QImage)), mRGBDContainer, SLOT(OnRGBD(QImage,QImage)), Qt::QueuedConnection);

    mLogic->EnablePointCloud(mActPointCloud->isChecked());
    mLogic->EnableRGBD(mActRGBDView->isChecked());
}

std::string MainWindow::GetChildObjName(QObject* obj) const
{
    // removing the 'm' in start of them
    const QString& name = obj->objectName();
    if( name.startsWith('m') )
        return name.right(name.length() - 1).toStdString();
    else
        return name.toStdString();
}

void MainWindow::CreateActions()
{
    mActQuit = new QAction(QIcon::fromTheme("application-exit"),tr("Quit"), this);
    SET_QTOBJECTNAME(mActQuit);
    mActQuit->setStatusTip(tr("Closes the application"));
    connect(mActQuit, SIGNAL(triggered()), this, SLOT(close()));
    
    mActPointCloud = new QAction(tr("Point Cloud"), this);
    SET_QTOBJECTNAME(mActPointCloud);
    InitCheckableAction(mActPointCloud, tr("Shows the current point cloud"));
    
    mActLogView = new QAction(tr("View Logs"), this);
    SET_QTOBJECTNAME(mActLogView);
    InitCheckableAction(mActLogView, tr("Shows logs window"));
    
    mActRGBDView = new QAction(tr("View RGBD"), this);
    SET_QTOBJECTNAME(mActRGBDView);
    InitCheckableAction(mActRGBDView, tr("Shows current RGB image"));
    
    mActExecControlView = new QAction(tr("Execution Controller"), this);
    SET_QTOBJECTNAME(mActExecControlView);
    InitCheckableAction(mActExecControlView, tr("Shows application controller"));
    
    mActStatistics = new QAction(tr("Show Statistics"), this);
    SET_QTOBJECTNAME(mActStatistics);
    InitCheckableAction(mActStatistics, tr("Shows statistics window"));
}

void MainWindow::CreateMenus()
{
    // File menu
    mMenuFile = menuBar()->addMenu(tr("File"));
    SET_QTOBJECTNAME(mMenuFile);
    mMenuFile->addAction(mActQuit);
    
    // Settings menu
    mMenuSettings = menuBar()->addMenu(tr("Settings"));
    SET_QTOBJECTNAME(mMenuSettings);

    // Windows menu
    mMenuWindows = menuBar()->addMenu(tr("Windows"));
    SET_QTOBJECTNAME(mMenuWindows);
    mMenuWindows->addAction(mActRGBDView);
    mMenuWindows->addAction(mActLogView);
    mMenuWindows->addAction(mActPointCloud);
    mMenuWindows->addAction(mActExecControlView);
    
    // Help menu
    mMenuHelp = menuBar()->addMenu(tr("Help"));
    SET_QTOBJECTNAME(mMenuHelp);
    // help menu has it's own about, so I didn't declare it for the class
    QAction* aboutAction = new QAction(QIcon::fromTheme("help-about"), tr("About"), this);
    mMenuHelp->addAction(aboutAction);
    connect(aboutAction, SIGNAL(triggered()), this, SLOT(About()));
}

void MainWindow::CreateStatusbar()
{
//     statusBar()->showMessage(tr("Ready"));
//     mFovisStatLabel = new QLabel(this);
//     SET_QTOBJECTNAME(mFovisStatLabel);
//     mFovisStatLabel->setMinimumWidth(500);
//     
//     mKinectStatLabel = new QLabel(this);
//     SET_QTOBJECTNAME(mKinectStatLabel);
//     
//     statusBar()->addPermanentWidget(mKinectStatLabel);
//     statusBar()->addPermanentWidget(mFovisStatLabel);
//     
//     mKinectStatLabel->setText("Kinect: Not running");
//     mFovisStatLabel->setText("Fovis: Not running");
}

void MainWindow::CreateToolbars()
{
    mToolbarFile = addToolBar(tr("File"));
    SET_QTOBJECTNAME(mToolbarFile);
    mToolbarFile->addAction(mActQuit);
    
    mToolbarSettings = addToolBar(tr("Settings"));
    SET_QTOBJECTNAME(mToolbarSettings);
    
    mToolbarWindows = addToolBar(tr("Windows"));
    SET_QTOBJECTNAME(mToolbarWindows);
    mToolbarWindows->addAction(mActRGBDView);
    mToolbarWindows->addAction(mActLogView);
    mToolbarWindows->addAction(mActPointCloud);
    mToolbarWindows->addAction(mActExecControlView);
}

void MainWindow::InitControl(KWidgetBase* wid)
{
    wid->InitControl(mGuiPO->StartNode(GetChildObjName(wid)));
    wid->setAttribute(Qt::WA_DeleteOnClose, false);
}

void MainWindow::SetupUI()
{
    //First create widgets, later we will add them to mdi.
    mLogContainer = new LogContainer(this);
    SET_QTOBJECTNAME(mLogContainer);
    InitControl(mLogContainer);

    mRGBDContainer = new ImageContainer(this);
    SET_QTOBJECTNAME(mRGBDContainer);
    InitControl(mRGBDContainer);
    
    mExecControl = new ExecutionControl(this);
    SET_QTOBJECTNAME(mExecControl);
    InitControl(mExecControl);
    
    mStatisticsContainer = new StatisticsContainer(this);
    SET_QTOBJECTNAME(mStatisticsContainer);
    InitControl(mStatisticsContainer);
    
    mPCViewer = new PointCloudViewer(this);
    SET_QTOBJECTNAME(mPCViewer);
    mPCViewer->setAttribute(Qt::WA_DeleteOnClose, false);
    
    Qt::WindowFlags flags = Qt::Window | Qt::WindowMinimizeButtonHint | 
            Qt::WindowCloseButtonHint | Qt::WindowSystemMenuHint;

    Qt::WindowFlags imageViewFlags = Qt::Window | Qt::WindowMinimizeButtonHint |
            Qt::WindowCloseButtonHint | Qt::WindowSystemMenuHint;
    // Create MDI windows
    mMdiRGBDView = mMdiArea->addSubWindow(mRGBDContainer, flags);
    SET_QTOBJECTNAME(mMdiRGBDView);
    InitMdiSubWindow(mMdiRGBDView, tr("RGBD Viewer"), mActRGBDView, false);
    
    mMdiLogView = mMdiArea->addSubWindow(mLogContainer, flags);
    SET_QTOBJECTNAME(mMdiLogView);
    InitMdiSubWindow(mMdiLogView, tr("Log Viewer"), mActLogView);

    mMdiPointCloud = mMdiArea->addSubWindow(mPCViewer);
    SET_QTOBJECTNAME(mMdiPointCloud);
    InitMdiSubWindow(mMdiPointCloud, tr("PointCloud Viewer"), mActPointCloud, false);
    
    mMdiExecController = mMdiArea->addSubWindow(mExecControl, imageViewFlags);
    SET_QTOBJECTNAME(mMdiExecController);
    InitMdiSubWindow(mMdiExecController, tr("Execution Controller"), mActExecControlView);

    mMdiStatisticsContainer = mMdiArea->addSubWindow(mStatisticsContainer, imageViewFlags);
    SET_QTOBJECTNAME(mMdiStatisticsContainer);
    InitMdiSubWindow(mMdiStatisticsContainer, tr("Statistics"), mActStatistics);
}

void MainWindow::InitMdiSubWindow(QMdiSubWindow* wnd, const QString& title, QAction* act, bool addToList)
{
    ProgramOptions::Ptr wndpo = mGuiPO->StartNode(GetChildObjName(wnd));
    wnd->installEventFilter(this);
    wnd->setWindowTitle(title);
    wnd->setAttribute(Qt::WA_DeleteOnClose, false);
    
    if( wndpo->NodeExists("MinumumSize") )
        wnd->setMinimumSize(Utils::ReadSize(wndpo, "MinimumSize", wnd->minimumSize()));
    if( wndpo->NodeExists("MaximumSize") )
        wnd->setMaximumSize(Utils::ReadSize(wndpo, "MaximumSize", wnd->maximumSize()));
    if( wndpo->NodeExists("FixedSize") )
       wnd->setFixedSize(Utils::ReadSize(wndpo, "FixedSize", wnd->minimumSizeHint()));
    if( wndpo->NodeExists("Position") )
        wnd->move(Utils::ReadPoint(wndpo, "Position", QPoint(0, 0)));
    if( act )
    {
        act->isChecked() ? wnd->show() : wnd->hide();
        if( addToList )
            mWidActMap.insert(WidgetActionMap::value_type(wnd, act));
    }
}

void MainWindow::InitCheckableAction(QAction* act, const QString& tip)
{
    act->setStatusTip(tip);
    act->setCheckable(true);
    //act->setChecked(mGuiPO->GetBool((objectName() + "." + act->objectName()).toStdString() + ".Checked", true));
    act->setChecked(mGuiPO->GetBool(GetChildObjName(act) + ".Checked", true));
    
    if( act == mActRGBDView )
        connect(act, SIGNAL(triggered(bool)), this, SLOT(ToggleRGBD(bool)));
    else if( act == mActPointCloud )
        connect(act, SIGNAL(triggered(bool)), this, SLOT(TogglePointCloud(bool)));
    else
        connect(act, SIGNAL(triggered(bool)), this, SLOT(ToggleAction(bool)));
}

void MainWindow::SaveMdiSubWindow(QMdiSubWindow* wnd)
{
    Utils::WritePoint(mGuiPO, GetChildObjName(wnd) + ".Position", wnd->pos());
}

void MainWindow::SaveCheckableAction(QAction* act)
{
    mGuiPO->PutBool(GetChildObjName(act) + ".Checked", act->isChecked());
}

void MainWindow::ToggleAction(bool bShow)
{
    QAction* act = static_cast<QAction*>(MainWindow::sender());
    WidgetActionMap::right_const_iterator iter = mWidActMap.right.find(act);
    if( iter == mWidActMap.right.end() )
    {
        qDebug() << "(MainWindow::ToggleAction) Unknown sender: (objectName:" << act->objectName() << 
                    ", class:" << act->metaObject()->className() << ")";
        return;
    }
    QWidget* wid = iter->second;
    bShow ? wid->show() : wid->hide();
    mGuiPO->PutBool(GetChildObjName(wid) + ".Checked", bShow);
}

void MainWindow::ToggleRGBD(bool bShow)
{
    bShow ? mMdiRGBDView->show() : mMdiRGBDView->hide();
    mLogic->EnableRGBD(bShow);
}

void MainWindow::TogglePointCloud(bool bShow)
{
    bShow ? mMdiPointCloud->show() : mMdiPointCloud->hide();
    mLogic->EnablePointCloud(bShow);
}

void MainWindow::About()
{
    QMessageBox::about(this, tr("About KSRobot"), tr("This program is designed and implemented by Kourosh Sartipi\n" 
            "This is my M.Sc. thesis implementation."));
}

void MainWindow::OnLogicError(const QString& str)
{
    QMessageBox::critical(this, tr("Logic Error"), str);
    emit mExecControl->GuiStop();
}

void MainWindow::Cleanup()
{
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // Check if any window has been closed, if yes, toggle the corresponding action
    if( event->type() == QEvent::Close && obj->isWidgetType() )
    {
        if( obj == mMdiPointCloud )
        {
            mLogic->EnablePointCloud(false);
            bool oldVal = mActRGBDView->blockSignals(true);
            mActRGBDView->setChecked(false);
            mActRGBDView->blockSignals(oldVal);
            event->ignore();
            return true;
        }
        else if( obj == mMdiRGBDView )
        {
            mLogic->EnableRGBD(false);
            bool oldVal = mActPointCloud->blockSignals(true);
            mActPointCloud->setChecked(false);
            mActPointCloud->blockSignals(oldVal);
            event->ignore();
            return true;
        }
        else
        {
            WidgetActionMap::left_const_iterator iter = mWidActMap.left.find(static_cast<QWidget*>(obj));
            if( iter != mWidActMap.left.end() )
            {
                QAction* act = iter->second;
                bool oldVal = act->blockSignals(true);
                act->setChecked(false);
                act->blockSignals(oldVal);
                static_cast<QWidget*>(obj)->hide();
                event->ignore();
                
                return true;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    // Save settings here!
    mLogic->OnStop();
    
    SaveCheckableAction(mActRGBDView);
    SaveCheckableAction(mActExecControlView);
    SaveCheckableAction(mActLogView);
    SaveCheckableAction(mActPointCloud);
    
    SaveMdiSubWindow(mMdiStatisticsContainer);
    SaveMdiSubWindow(mMdiExecController);
    SaveMdiSubWindow(mMdiLogView);
    SaveMdiSubWindow(mMdiPointCloud);

    QWidget::closeEvent(event);
}

void MainWindow::InstallMsgHandler()
{
    s_windowLogReceiverList.append(this);
    if( s_bHandlerInitialized == false )
    {
        s_prevMsgHandler = qInstallMsgHandler(&MainWindow::QtMessageHandler);
        s_bHandlerInitialized = true;
    }
}

void MainWindow::UnInstallMsgHandler()
{
    for(int i = 0; i < s_windowLogReceiverList.size(); i++)
    {
        if( s_windowLogReceiverList[i] == this )
        {
            s_windowLogReceiverList.remove(i);
            if( s_windowLogReceiverList.size() == 0 )
            {
                // install the old handler, and set initialized to false
                s_bHandlerInitialized = false;
                qInstallMsgHandler(s_prevMsgHandler);
            }
            break;
        }
    }
}

void MainWindow::QtMessageHandler(QtMsgType mt, const char* msg)
{
    for(QVector<MainWindow*>::iterator iter = MainWindow::s_windowLogReceiverList.begin();
        iter != MainWindow::s_windowLogReceiverList.end(); iter++)
    {
        MainWindow* wnd = *iter;
        LogContainer* logger = wnd->mLogContainer;
        LogType lt = LT_LOG;
        
        switch(mt)
        {
            case QtDebugMsg:
                lt = LT_DEBUG;
                break;
            case QtWarningMsg:
                lt = LT_WARNING;
                break;
            case QtCriticalMsg:
            case QtFatalMsg:
                lt = LT_ERROR;
                // in case of fatal we should abort,
                break;
        }
        logger->Log(lt, msg);
    }
    
    if( s_prevMsgHandler != 0 )
        s_prevMsgHandler(mt, msg);
}
