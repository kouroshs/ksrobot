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

#include <QDebug>
#include <QMessageBox>
#include <QLabel>
#include <QVector>
#include <QMetaObject>
#include <QResizeEvent>
#include <QSignalMapper>

using std::cout;
using std::flush;

using namespace KSRobot::gui;
using namespace KSRobot::utils;

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
    
    showMaximized();
}

void MainWindow::CreateActions()
{
    mActQuit = new QAction(QIcon::fromTheme("application-exit"),tr("Quit"), this);
    SET_QTOBJECTNAME(mActQuit);
    mActQuit->setStatusTip(tr("Closes the application"));
    connect(mActQuit, SIGNAL(triggered()), this, SLOT(close()));
    mActConfig = new QAction(tr("Config"), this);
    SET_QTOBJECTNAME(mActConfig);
    mActConfig->setStatusTip(tr("Opens configuration dialog"));
    connect(mActConfig, SIGNAL(triggered(bool)), this, SLOT(ShowConfig(bool)));
    
    mActPointCloud = new QAction(tr("Point Cloud"), this);
    SET_QTOBJECTNAME(mActPointCloud);
    InitCheckableAction(mActPointCloud, tr("Shows the current point cloud"));
    
    mActDepthView = new QAction(tr("View Depth"), this);
    SET_QTOBJECTNAME(mActDepthView);
    InitCheckableAction(mActDepthView, tr("Shows the current depth image"));
    
    mActLogView = new QAction(tr("View Logs"), this);
    SET_QTOBJECTNAME(mActLogView);
    InitCheckableAction(mActLogView, tr("Shows logs window"));
    
    mActMapView = new QAction(tr("View Map"), this);
    SET_QTOBJECTNAME(mActMapView);
    InitCheckableAction(mActMapView, tr("Shows the current created map"));
    
    mActRgbView = new QAction(tr("View RGB"), this);
    SET_QTOBJECTNAME(mActRgbView);
    InitCheckableAction(mActRgbView, tr("Shows current RGB image"));
    
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
    mMenuSettings->addAction(mActConfig);

    // Windows menu
    mMenuWindows = menuBar()->addMenu(tr("Windows"));
    SET_QTOBJECTNAME(mMenuWindows);
    mMenuWindows->addAction(mActRgbView);
    mMenuWindows->addAction(mActDepthView);
    mMenuWindows->addAction(mActLogView);
    mMenuWindows->addAction(mActPointCloud);
    mMenuWindows->addAction(mActMapView);
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
    mToolbarSettings->addAction(mActConfig);
    
    mToolbarWindows = addToolBar(tr("Windows"));
    SET_QTOBJECTNAME(mToolbarWindows);
    mToolbarWindows->addAction(mActRgbView);
    mToolbarWindows->addAction(mActDepthView);
    mToolbarWindows->addAction(mActLogView);
    mToolbarWindows->addAction(mActPointCloud);
    mToolbarWindows->addAction(mActMapView);
    mToolbarWindows->addAction(mActExecControlView);
}

void MainWindow::InitControl(KWidgetBase* wid)
{
    wid->InitControl(mGuiPO);
    wid->setAttribute(Qt::WA_DeleteOnClose, false);
}

void MainWindow::SetupUI()
{
    //First create widgets, later we will add them to mdi.
    mConfigDialog = new ConfigDialog(this, 0, mGuiPO);
    SET_QTOBJECTNAME(mConfigDialog);
    connect(mConfigDialog, SIGNAL(ConfigChanged()), this, SLOT(ConfigChanged()));
    
    mLogContainer = new LogContainer(this);
    SET_QTOBJECTNAME(mLogContainer);
    InitControl(mLogContainer);

    mRgbContainer = new ImageContainer(this);
    SET_QTOBJECTNAME(mRgbContainer);
    InitControl(mRgbContainer);
    
    mDepthContainer = new ImageContainer(this);
    SET_QTOBJECTNAME(mDepthContainer);
    InitControl(mDepthContainer);
    
    mExecControl = new ExecutionControl(this);
    SET_QTOBJECTNAME(mExecControl);
    InitControl(mExecControl);
    
    mStatisticsContainer = new StatisticsContainer(this);
    SET_QTOBJECTNAME(mStatisticsContainer);
    InitControl(mStatisticsContainer);
    
    qDebug() << "(MainWindow::SetupUI) No containers for PointCloud and map.";
    
    Qt::WindowFlags flags = Qt::Window | Qt::WindowMinimizeButtonHint | 
            Qt::WindowCloseButtonHint | Qt::WindowSystemMenuHint;

    Qt::WindowFlags imageViewFlags = Qt::Window | Qt::WindowMinimizeButtonHint |
            Qt::WindowCloseButtonHint | Qt::WindowSystemMenuHint;
    // Create MDI windows
    mMdiRgbView = mMdiArea->addSubWindow(mRgbContainer, imageViewFlags);
    SET_QTOBJECTNAME(mMdiRgbView);
    InitMdiSubWindow(mMdiRgbView, tr("RGB Viewer"), mActRgbView);
    
    mMdiDepthView = mMdiArea->addSubWindow(mDepthContainer, imageViewFlags);
    SET_QTOBJECTNAME(mMdiDepthView);
    InitMdiSubWindow(mMdiDepthView, tr("Depth Viewer"), mActDepthView);
    
    mMdiLogView = mMdiArea->addSubWindow(mLogContainer, flags);
    SET_QTOBJECTNAME(mMdiLogView);
    InitMdiSubWindow(mMdiLogView, tr("Log Viewer"), mActLogView);

    mMdiMapView = mMdiArea->addSubWindow(new QLabel("MapView", this));
    SET_QTOBJECTNAME(mMdiMapView);
    InitMdiSubWindow(mMdiMapView, tr("Map Viewer"), mActMapView);
    
    mMdiPointCloud = mMdiArea->addSubWindow(new QLabel("PointCloudView", this));
    SET_QTOBJECTNAME(mMdiPointCloud);
    InitMdiSubWindow(mMdiPointCloud, tr("PointCloud Viewer"), mActPointCloud);
    
    mMdiExecController = mMdiArea->addSubWindow(mExecControl, imageViewFlags);
    SET_QTOBJECTNAME(mMdiExecController);
    InitMdiSubWindow(mMdiExecController, tr("Execution Controller"), mActExecControlView);

    mMdiStatisticsContainer = mMdiArea->addSubWindow(mStatisticsContainer, imageViewFlags);
    SET_QTOBJECTNAME(mMdiStatisticsContainer);
    InitMdiSubWindow(mMdiStatisticsContainer, tr("Statistics"), mActStatistics);
}

void MainWindow::InitMdiSubWindow(QMdiSubWindow* wnd, const QString& title, QAction* act)
{
    ProgramOptions::Ptr wndpo = mGuiPO->StartNode(wnd->objectName().toStdString());
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
        mWidActMap.insert(WidgetActionMap::value_type(wnd, act));
    }
}

void MainWindow::InitCheckableAction(QAction* act, const QString& tip)
{
    act->setStatusTip(tip);
    act->setCheckable(true);
    act->setChecked(mGuiPO->GetBool(act->objectName().toStdString() + ".Checked", true));
    connect(act, SIGNAL(triggered(bool)), this, SLOT(ToggleAction(bool)));
}

void MainWindow::SaveMdiSubWindow(QMdiSubWindow* wnd)
{
    Utils::WritePoint(mGuiPO, wnd->objectName().toStdString() + 
                        ".Position", wnd->pos());
    
    //qDebug() << "(MainWindow::SaveMdiSubWindow) " << wnd->objectName();
}

void MainWindow::SaveCheckableAction(QAction* act)
{
    //qDebug() << "(MainWindow::SaveCheckableAction) " << act->objectName();
    mGuiPO->PutBool(act->objectName().toStdString() + ".Checked", act->isChecked());
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
    mGuiPO->PutBool(wid->objectName().toStdString() + ".Checked", bShow);
}

void MainWindow::ShowConfig(bool bShow)
{
    (void)bShow;
    qDebug() << "(MainWindow::ShowConfig) Method not implemented.";
}


void MainWindow::About()
{
    QMessageBox::about(this, tr("About KSRobot"), tr("This program is designed and implemented by Kourosh Sartipi\n" 
            "This is my M.Sc. thesis implementation."));
}

void MainWindow::Cleanup()
{
}

void MainWindow::ConfigChanged()
{
    qDebug() << "(MainWindow::ConfigChanged) Implement this method.";
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // Check if any window has been closed, if yes, toggle the corresponding action
    if( event->type() == QEvent::Close && obj->isWidgetType() )
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
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    // Save settings here!
    SaveCheckableAction(mActRgbView);
    SaveCheckableAction(mActConfig);
    SaveCheckableAction(mActDepthView);
    SaveCheckableAction(mActExecControlView);
    SaveCheckableAction(mActLogView);
    SaveCheckableAction(mActMapView);
    SaveCheckableAction(mActPointCloud);
    
    SaveMdiSubWindow(mMdiStatisticsContainer);
    SaveMdiSubWindow(mMdiDepthView);
    SaveMdiSubWindow(mMdiExecController);
    SaveMdiSubWindow(mMdiLogView);
    SaveMdiSubWindow(mMdiMapView);
    SaveMdiSubWindow(mMdiPointCloud);
    SaveMdiSubWindow(mMdiRgbView);

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
