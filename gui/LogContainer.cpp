#include <gui/LogContainer.h>
#include <gui/Utils.h>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace KSRobot::gui;

QMap<QString, int> LogContainer::m_sColorMap;

LogContainer::LogContainer(QWidget* parent, Qt::WindowFlags f): KWidgetBase(parent, f)
{
}

LogContainer::~LogContainer()
{
}

void LogContainer::InitControl(KSRobot::common::ProgramOptions::Ptr po)
{
    KSRobot::gui::KWidgetBase::InitControl(po);
    mTextBox = new QTextEdit(this);
    SET_QTOBJECTNAME(mTextBox);
    
    mBtnClearText = new QPushButton(QIcon::fromTheme("edit-clear"), tr("Clear Logs"), this);
    SET_QTOBJECTNAME(mBtnClearText);
    
    connect(mBtnClearText, SIGNAL(clicked()), this, SLOT(OnClearTextBtnPress()));
    
    QVBoxLayout* rootLoggerLayout = new QVBoxLayout(this);
    SET_QTOBJECTNAME(rootLoggerLayout);
    
    QWidget* horizLoggerWidget = new QWidget(this);
    SET_QTOBJECTNAME(horizLoggerWidget);
    
    QHBoxLayout* btnClearLoggerLayout = new QHBoxLayout(horizLoggerWidget);
    SET_QTOBJECTNAME(btnClearLoggerLayout);
    
    btnClearLoggerLayout->addStretch(0);
    btnClearLoggerLayout->addWidget(mBtnClearText);
    
    rootLoggerLayout->addWidget(mTextBox);
    rootLoggerLayout->addWidget(horizLoggerWidget);
    
    mTextBox->setReadOnly(true);
    mTextBox->setLineWrapMode(QTextEdit::NoWrap);
    //default colors
    mDefaultTextColor = Qt::black;
    
    mColorMap[LT_LOG] = Qt::black;
    mChannelEnabled[LT_LOG] = true;
    mPreString[LT_LOG] = "LOG: ";
    
    mColorMap[LT_DEBUG] = Qt::darkGray;
    mChannelEnabled[LT_DEBUG] = true;
    mPreString[LT_DEBUG] = "DBG: ";
    
    mColorMap[LT_WARNING] = Qt::darkYellow;
    mChannelEnabled[LT_WARNING] = true;
    mPreString[LT_WARNING] = "WRN: ";
    
    mColorMap[LT_ERROR] = Qt::red;
    mChannelEnabled[LT_ERROR] = true;
    mPreString[LT_ERROR] = "ERR: ";
    
    setMinimumSize(500, 100);
}


QColor LogContainer::GetDefaultTextColor() const
{
    return mDefaultTextColor;
}

void LogContainer::SetDefaultTextColor(QColor color)
{
    mDefaultTextColor = color;
}

QColor LogContainer::GetLogLevelColor(LogType lt) const
{
    return mColorMap[lt];
}

void LogContainer::SetLogLevelColor(LogType lt, QColor color)
{
    mColorMap[lt] = color;
}

void LogContainer::EnableLogChannel(LogType lt)
{
    mChannelEnabled[lt] = true;
}

void LogContainer::DisableLogChannel(LogType lt)
{
    mChannelEnabled[lt] = false;
}

void LogContainer::SetPreString(LogType lt, QString str)
{
    mPreString[lt] = str;
}

QString LogContainer::GetPreString(LogType lt) const
{
    return mPreString[lt];
}

bool LogContainer::IsLogChannelEnabled(LogType lt)
{
    return mChannelEnabled[lt];
}

void LogContainer::Log(LogType lt, const QString& str)
{
    if( !IsLogChannelEnabled(lt) )
        return;
    
    mTextBox->setTextColor(mColorMap[lt]);
    mTextBox->append(mPreString[lt] + str);
    mTextBox->moveCursor(QTextCursor::End);
    mTextBox->moveCursor(QTextCursor::StartOfLine);
}

void LogContainer::OnClearTextBtnPress()
{
    mTextBox->clear();
}
