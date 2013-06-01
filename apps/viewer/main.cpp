/**
 * @file   main.cpp
 * @Author Kourosh Sartipi (kourosh.sartipi@gmail.com)
 * @date   March, 2013
 * @brief  main entry point of the program.
 *
 */

#include <QApplication>
#include <gui/MainWindow.h>
#include <apps/viewer/VisualizerApplication.h>
#include <utils/ProgramOptions.h>

using namespace boost::property_tree;
using namespace std;

using namespace KSRobot;
using namespace gui;
using namespace common;

int main(int argc, char** argv)
{
    VisualizerApplication app(argc, argv);
    app.SetSettingsFileName("ksrobot-settings.xml");
    
    VisualizerApplication::setOrganizationDomain("ce.sharif.edu");
    VisualizerApplication::setOrganizationName("Sharif University of Technology");
    VisualizerApplication::setApplicationName("KSRobot");
    VisualizerApplication::setApplicationVersion("0.0.2");
    
    KSRobot::gui::MainWindow mainWin;
    mainWin.setObjectName("mainWin");
    mainWin.InitWindow(app.GetPO(), app.GetPO()->StartNode("GUI"));
    mainWin.show();
    return app.exec();
}
