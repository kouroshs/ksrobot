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

#include <common/ProgramOptions.h>


#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTcpSocket>
#include <QMessageBox>
#include <gui/Utils.h>
#include <QFormLayout>
//#include <cstdint>

class TempSocketWin : public QDialog
{
    Q_OBJECT
public:
    QLineEdit *edtHost, *edtPort;
    QPushButton *btnConnect, *btnDisconnect;
    
    QPushButton *btnForward, *btnBackward, *btnTurnL, *btnTurnR, *btnStop;
    QLineEdit *edtAmount, *edtSpeed;
    
    QTcpSocket* socket;
    
    TempSocketWin(QWidget* parent) : QDialog(parent)
    {
        setWindowTitle("Dosconnected");
        
        socket = new QTcpSocket(this);
        connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(OnSocketError()));
        connect(socket, SIGNAL(connected()), this, SLOT(OnSocketConnect()));
        connect(socket, SIGNAL(disconnected()), this, SLOT(OnSocketDisconnect()));

        btnStop = new QPushButton("Stop", this);
        btnForward = new QPushButton("Forward", this);
        btnBackward = new QPushButton("Backward", this);
        btnTurnL = new QPushButton("Turn Left", this);
        btnTurnR = new QPushButton("Turn Right", this);
        
        edtAmount = new QLineEdit("0", this);
        edtSpeed = new QLineEdit("0", this);
    
        edtHost = new QLineEdit(this);
        edtPort = new QLineEdit(this);
        btnConnect = new QPushButton("Connect", this);
        btnDisconnect = new QPushButton("Disconnect", this);
        
        QHBoxLayout* top = new QHBoxLayout(this);
        SET_QTOBJECTNAME(top);
        QVBoxLayout* btns = new QVBoxLayout();
        SET_QTOBJECTNAME(btns);
        QVBoxLayout* edts = new QVBoxLayout();
        SET_QTOBJECTNAME(edts);
        
        QFormLayout* l = new QFormLayout();
        l->addRow("Host:", edtHost);
        l->addRow("Port:", edtPort);
        l->addRow(btnConnect);
        l->addRow(btnDisconnect);
        
//         QVBoxLayout* cnl = new QVBoxLayout;
//         cnl->addWidget(btnConnect);
//         cnl->addWidget(btnDisconnect);
        
        top->addLayout(l);
//         top->addLayout(cnl);
        top->addLayout(btns);
        top->addLayout(edts);
        
        btns->addWidget(btnStop);
        btns->addWidget(btnForward);
        btns->addWidget(btnBackward);
        btns->addWidget(btnTurnL);
        btns->addWidget(btnTurnR);
        
        edts->addWidget(new QLabel("Amount:", this));
        edts->addWidget(edtAmount);
        edts->addWidget(new QLabel("Speed:", this));
        edts->addWidget(edtSpeed);
        
        connect(btnConnect, SIGNAL(clicked()), this, SLOT(OnConnect()));
        connect(btnDisconnect, SIGNAL(clicked()), this, SLOT(OnDisconnect()));
        
        connect(btnStop, SIGNAL(clicked()), this, SLOT(OnStop()));
        connect(btnForward, SIGNAL(clicked()), this, SLOT(OnForward()));
        connect(btnBackward, SIGNAL(clicked()), this, SLOT(OnBackward()));
        connect(btnTurnL, SIGNAL(clicked()), this, SLOT(OnTurnL()));
        connect(btnTurnR, SIGNAL(clicked()), this, SLOT(OnTurnR()));        
        
    }
    
    virtual ~TempSocketWin(){;}
    
    void SendCommand(const std::string& data)
    {
        bool ok;
        int amount = edtAmount->text().toInt(&ok);
        if( !ok )
        {
            setWindowTitle("ERROR: Invalid amount");
            return;
        }
        float speed = edtSpeed->text().toFloat(&ok);
        if( !ok )
        {
            setWindowTitle("ERROR: Invalid speed");
            return;
        }
        std::stringstream ss;
        ss << data << "," << amount << ","<< speed;
        
        char bytes[1024];
        int* ptrMsgId = (int*)bytes;
        int* ptrMsgLen = (int*)(bytes + 4);
        *ptrMsgId = 0; // Message ID - 4 bytes
        *ptrMsgLen = ss.str().length(); // Message Length
        
        //TODO: This is unsafe
        memcpy(bytes + 8, ss.str().c_str(), ss.str().length());
        
        socket->write(bytes, ss.str().length() + 8);
    }
    
public slots:
    void OnSocketConnect()
    {
        setWindowTitle("Connected");
    }

    void OnSocketDisconnect()
    {
        setWindowTitle("Disconnected");
    }
    
    void OnConnect()
    {
        socket->connectToHost(edtHost->text(), edtPort->text().toInt());
    }
    
    void OnDisconnect()
    {
        socket->disconnect();
    }
    
    void OnSocketError()
    {
        QMessageBox::critical(this, "Socket Error", "Socket connection failed. Reason: " + socket->errorString());
        
    }
    
    void OnStop()
    {
        std::cout << "Hello!\n";
    }
    
    void OnForward()
    {
        SendCommand("forward");
    }
    
    void OnBackward()
    {
        SendCommand("backward");
    }
    
    void OnTurnL()
    {
        SendCommand("turnl");
    }
    
    void OnTurnR()
    {
        SendCommand("turnr");
    }
    
    
};


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

    void InitWindow(KSRobot::common::ProgramOptions::Ptr root, KSRobot::common::ProgramOptions::Ptr poGUI);
    
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

