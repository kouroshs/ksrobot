#include <iostream>
#include <roboctrl/RobXControlDialog.h>
#include <roboctrl/ThreadedRoboCtrl.h>
#include <QApplication>

using namespace KSRobot;
using namespace roboctrl;
using namespace std;

int main(int argc, char** argv)
{
    ThreadedRoboCtrl thrd;
    
    //thrd.SetDevice("/dev/ttyUSB1", "/dev/ttyUSB0");
    //thrd.SetDevice("/dev/ttyUSB0", "/dev/ttyUSB1");
    thrd.SetDevice("/dev/ttyUSB0");
    thrd.StartCommThread();

    std::cout << "111\n" << std::flush;
//     thrd.PutCommandToQueue(RobotCommand::Turn(-90, 1));
//     thrd.PutCommandToQueue(RobotCommand::Turn(-90, 1));
//     thrd.PutCommandToQueue(RobotCommand::Turn(-90, 1));
//     thrd.PutCommandToQueue(RobotCommand::Turn(-90, 1));
    //thrd.PutCommandToQueue(RobotCommand::Move(40, 1));

        thrd.PutCommandToQueue(RobotCommand::Move(20, 1));
        thrd.PutCommandToQueue(RobotCommand::Turn(30, 1));
    
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(76, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(743, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(304, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(344, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(35, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(314, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(292, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(510, 4));
// 
//     thrd.PutCommandToQueue(RobotCommand::Move(-510, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(35, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(-314, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(304, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(-344, 4));
//     thrd.PutCommandToQueue(RobotCommand::TurnTo(76, 1));
//     thrd.PutCommandToQueue(RobotCommand::Move(-743, 4));
//     
    
    std::cout << "222\n" << std::flush;
    
    
//    thrd.PutCommandToQueue(RobotCommand::Move(100, 3.0f));
//    thrd.PutCommandToQueue(RobotCommand::Turn(90, 1.0f));
//    thrd.PutCommandToQueue(RobotCommand::Turn(90, 1.0f));

    std::cout << "333\n" << std::flush;
    
    thrd.WaitForAllCommands();
    
    std::cout << "444\n" << std::flush;
    
    thrd.StopCommThread();
    
    
    return 0;
}

