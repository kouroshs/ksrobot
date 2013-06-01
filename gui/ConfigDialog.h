#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>
#include <gui/ImageContainer.h>
#include <common/ProgramOptions.h>

namespace KSRobot
{
namespace gui
{

class ConfigDialog : public QDialog
{
    Q_OBJECT
public:
    //TODO: Remove this, and setup Execution Control
    explicit ConfigDialog(QWidget* parent, Qt::WindowFlags f, 
                          KSRobot::common::ProgramOptions::Ptr po);
    virtual ~ConfigDialog();
    
signals:
    void                                        ConfigChanged();
private:
    KSRobot::common::ProgramOptions::Ptr         mGuiPO;
    
};

} // end namespace gui
} // end namespace KSRobot

#endif // CONFIGDIALOG_H
