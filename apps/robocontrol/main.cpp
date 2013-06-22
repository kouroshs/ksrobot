#include <iostream>
#include <gui/RobXControlDialog.h>
#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    KSRobot::gui::RobXControlDialog dlg;
    
    dlg.show();
    
    return app.exec();
}

