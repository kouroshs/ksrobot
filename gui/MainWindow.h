#ifndef KSROBOT_GUI_MAINWINDOW_H
#define KSROBOT_GUI_MAINWINDOW_H

/**
 * @file   MainWindow.h
 * @Author Kourosh Sartipi (kourosh.sartipi@gmail.com)
 * @date   March, 2013
 * @brief  Header file of the main window of GUI.
 *
 * This is the header file for the main window of KSRobot GUI. It defines
 * various functions and signals and slots needed to other windows or threads
 * to communicate with the gui.
 */

#include <QMainWindow>

#include <QAction>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QMenu>
#include <QMenuBar>
#include <QLabel>
#include <QStatusBar>
#include <QToolBar>
#include <QVector>
#include <QEvent>
#include <QThread>
#include <QTimer>
#include <QSignalMapper>

#include <boost/bimap.hpp>

#include <gui/ImageContainer.h>
#include <gui/LogContainer.h>
#include <gui/ConfigDialog.h>
#include <gui/ExecutionControl.h>
#include <gui/StatisticsContainer.h>

#include <utils/ProgramOptions.h>

namespace KSRobot
{
namespace gui
{

class MainWindow : public QMainWindow
{
    Q_OBJECT;
public:
    MainWindow();
    virtual ~MainWindow();

    void InitWindow(KSRobot::utils::ProgramOptions::Ptr root, KSRobot::utils::ProgramOptions::Ptr poGUI);
    
private slots:
    void                                ToggleAction(bool);
    void                                ShowConfig(bool);
    void                                About();
    
    void                                ConfigChanged();
    
private:
    void                                CreateActions();
    void                                CreateMenus();
    void                                CreateToolbars();
    void                                CreateStatusbar();
    void                                SetupUI();
    
    
    std::string                         GetChildObjName(QObject* obj) const;
    
    void                                Cleanup();
    
    void                                InitMdiSubWindow(QMdiSubWindow* wnd, const QString& title, QAction* act);
    void                                SaveMdiSubWindow(QMdiSubWindow* wnd);
    void                                InitCheckableAction(QAction* act, const QString& top);
    void                                SaveCheckableAction(QAction* act);
    
    void                                InitControl(KWidgetBase* wid);
protected:
    // events
    virtual bool                        eventFilter(QObject* obj, QEvent* event);
    virtual void                        closeEvent(QCloseEvent *event);
// now data mebers
private:
    // Actions
    QAction*                            mActQuit;
    QAction*                            mActConfig;
    QAction*                            mActDepthView;
    QAction*                            mActRgbView;
    QAction*                            mActLogView;
    QAction*                            mActPointCloud;
    QAction*                            mActMapView;
    QAction*                            mActExecControlView;
    QAction*                            mActStatistics;
    
    // Menus
    QMenu*                              mMenuFile;
    QMenu*                              mMenuSettings;
    QMenu*                              mMenuWindows;
    QMenu*                              mMenuHelp;
    
    // Toolbars
    QToolBar*                           mToolbarFile;
    QToolBar*                           mToolbarSettings;
    QToolBar*                           mToolbarWindows;
    
    // MDI area
    QMdiArea*                           mMdiArea;
    //MDI Windows
    QMdiSubWindow*                      mMdiRgbView;
    QMdiSubWindow*                      mMdiDepthView;
    QMdiSubWindow*                      mMdiLogView;
    QMdiSubWindow*                      mMdiPointCloud;
    QMdiSubWindow*                      mMdiMapView;
    QMdiSubWindow*                      mMdiExecController;
    QMdiSubWindow*                      mMdiStatisticsContainer;
    
    ConfigDialog*                       mConfigDialog;
    LogContainer*                       mLogContainer;
    ImageContainer*                     mRgbContainer;
    ImageContainer*                     mDepthContainer;
    
    ExecutionControl*                   mExecControl;
    StatisticsContainer*                mStatisticsContainer;
    
    QLabel*                             mFovisStatLabel;
    QLabel*                             mKinectStatLabel;
    
    typedef boost::bimap<QWidget*, QAction*>   WidgetActionMap;
    WidgetActionMap                     mWidActMap;
    
    
    KSRobot::utils::ProgramOptions::Ptr mGuiPO;
    KSRobot::utils::ProgramOptions::Ptr mRootPO;
    
private:
    static QVector<MainWindow*>         s_windowLogReceiverList;
    static bool                         s_bHandlerInitialized;
    static QtMsgHandler                 s_prevMsgHandler;
    
    void                                InstallMsgHandler();
    void                                UnInstallMsgHandler();
    static void                         QtMessageHandler(QtMsgType, const char*);
}; // end class MainWindow

} // end namespace gui
} // end namespace KSRobot

#endif //KSROBOT_GUI_MAINWINDOW_H

