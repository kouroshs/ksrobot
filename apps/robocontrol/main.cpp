#include <iostream>
#include <roboctrl/RobXControlDialog.h>
#include <QApplication>

#include <common/Defenitions.h>

void MyFunc()
{
    KSRobot::common::PrintStackTrace();
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    
//     KSRobot::common::ProgramOptions::Ptr po(new KSRobot::common::ProgramOptions());
//     po->LoadFromFile("roboctrl-settings.xml");
    
    //KSRobot::common::RegisterDebugModeStackTracePrinter();
    KSRobot::roboctrl::RobXControlDialog dlg;
//    dlg.ReadSettings(po);
    dlg.show();
    
    int ret = app.exec();
//    po->SaveToFile("roboctrl-settings.xml");
    return ret;
}

