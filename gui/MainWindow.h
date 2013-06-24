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

#include <QVector>
#include <QMainWindow>
#include <QImage>
#include <boost/bimap.hpp>

#include <common/ProgramOptions.h>

//Forward declarations
class QAction;
class QMdiArea;
class QMdiSubWindow;
class QMenu;
class QEvent;
class QLabel;
class QToolBar;

namespace KSRobot
{
namespace gui
{

class ImageContainer;
class LogContainer;
class ExecutionControl;
class StatisticsContainer;
class PointCloudViewer;
class LogicBridge;
class KWidgetBase;

class MainWindow : public QMainWindow
{
    Q_OBJECT;
public:
    MainWindow();
    virtual ~MainWindow();

    void InitWindow(KSRobot::common::ProgramOptions::Ptr root, KSRobot::common::ProgramOptions::Ptr poGUI);
    
private slots:
    void                                ToggleAction(bool);
    void                                ToggleRGBD(bool);
    void                                TogglePointCloud(bool);
    void                                About();
    
    void                                OnLogicError(const QString& str);
private:
    void                                CreateActions();
    void                                CreateMenus();
    void                                CreateToolbars();
    void                                CreateStatusbar();
    void                                SetupUI();
    void                                SetupLogic();
    
    std::string                         GetChildObjName(QObject* obj) const;
    
    void                                Cleanup();
    
    void                                InitMdiSubWindow(QMdiSubWindow* wnd, const QString& title, QAction* act, bool addToList = true);
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
    QAction*                            mActRGBDView;
    QAction*                            mActLogView;
    QAction*                            mActPointCloud;
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
    QMdiSubWindow*                      mMdiRGBDView;
    QMdiSubWindow*                      mMdiLogView;
    QMdiSubWindow*                      mMdiPointCloud;
    QMdiSubWindow*                      mMdiExecController;
    QMdiSubWindow*                      mMdiStatisticsContainer;
    
    LogContainer*                       mLogContainer;
    ImageContainer*                     mRGBDContainer;
    PointCloudViewer*                   mPCViewer;
    
    ExecutionControl*                   mExecControl;
    StatisticsContainer*                mStatisticsContainer;
    
    QLabel*                             mFovisStatLabel;
    QLabel*                             mKinectStatLabel;
    
    typedef boost::bimap<QWidget*, QAction*>   WidgetActionMap;
    WidgetActionMap                     mWidActMap;
    LogicBridge*                        mLogic;
    
    
    KSRobot::common::ProgramOptions::Ptr mGuiPO;
    KSRobot::common::ProgramOptions::Ptr mRootPO;
    
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

