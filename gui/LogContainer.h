#ifndef LOGCONTAINER_H
#define LOGCONTAINER_H

#include <QTextEdit>
#include <QPushButton>
#include <QMap>
#include <utils/ProgramOptions.h>
#include <gui/KWidgetBase.h>

namespace KSRobot
{
namespace gui
{

enum LogType
{
    LT_LOG,
    LT_DEBUG,
    LT_WARNING,
    LT_ERROR
};

class LogContainer : public KWidgetBase
{
    Q_OBJECT
public:
    explicit LogContainer(QWidget* parent = 0, Qt::WindowFlags f = 0);
    virtual ~LogContainer();
    
    virtual void InitControl(common::ProgramOptions::Ptr po);
    
    QColor                              GetDefaultTextColor() const;
    void                                SetDefaultTextColor(QColor color);
    
    QColor                              GetLogLevelColor(LogType lt) const;
    void                                SetLogLevelColor(LogType lt, QColor color);
    
    void                                Log(LogType lt, const QString& str);
    
    void                                EnableLogChannel(LogType lt);
    void                                DisableLogChannel(LogType lt);
    bool                                IsLogChannelEnabled(LogType lt);
    
    QString                             GetPreString(LogType lt) const;
    void                                SetPreString(LogType lt, QString str);

private slots:
    void                                OnClearTextBtnPress();
    
private:
    QTextEdit*                          mTextBox;
    QPushButton*                        mBtnClearText;
    
    QMap<LogType, QColor>               mColorMap;
    QMap<LogType, bool>                 mChannelEnabled;
    QMap<LogType, QString>              mPreString;
    
    QColor                              mDefaultTextColor;
    
    static QMap<QString, int>           m_sColorMap;
};

} // end namespace gui
} // end namespace KSRobot

#endif // LOGCONTAINER_H
