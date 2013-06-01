#ifndef VISUALIZERAPPLICATION_H
#define VISUALIZERAPPLICATION_H

#include <QApplication>
#include <utils/ProgramOptions.h>

class VisualizerApplication : public QApplication
{
public:
    VisualizerApplication(int& argc, char** argv, int oa = ApplicationFlags);
    virtual ~VisualizerApplication();
    
    std::string                         GetSettingsFileName() const { return mSettingsFileName; }
    void                                SetSettingsFileName(const std::string& fn);
    
    KSRobot::common::ProgramOptions::Ptr GetPO() { return mPO; }
    
private:
    KSRobot::common::ProgramOptions::Ptr mPO;
    std::string                         mSettingsFileName;
};

#endif // VISUALIZERAPPLICATION_H

