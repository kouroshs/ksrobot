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
    KSRobot::common::RegisterDebugModeStackTracePrinter();
    KSRobot::roboctrl::RobXControlDialog dlg;
    
    dlg.show();
    
    return app.exec();
}

