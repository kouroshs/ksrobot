#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include <QDialog>
#include <gui/ImageContainer.h>
#include <utils/ProgramOptions.h>

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
                          KSRobot::utils::ProgramOptions::Ptr po);
    virtual ~ConfigDialog();
    
signals:
    void                                        ConfigChanged();
private:
    KSRobot::utils::ProgramOptions::Ptr         mGuiPO;
    
};

} // end namespace gui
} // end namespace KSRobot

#endif // CONFIGDIALOG_H
