#include <iostream>
#include <roboctrl/RobXControlDialog.h>
#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    KSRobot::roboctrl::RobXControlDialog dlg;
    dlg.show();
    
    return app.exec();
}

