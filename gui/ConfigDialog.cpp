#include <gui/ConfigDialog.h>

using namespace KSRobot::gui;

ConfigDialog::ConfigDialog(QWidget* parent, Qt::WindowFlags f, KSRobot::utils::ProgramOptions::Ptr po) :
    QDialog(parent, f), mGuiPO(po)
{
    
}

ConfigDialog::~ConfigDialog()
{

}
