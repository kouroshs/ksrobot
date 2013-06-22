set(GUI_SRCS PointCloudViewer.cpp RobXControlDialog.cpp RobXControl.cpp LogicBridge.cpp 
             KWidgetBase.cpp StatisticsContainer.cpp ExecutionControl.cpp 
             Utils.cpp MainWindow.cpp ImageContainer.cpp 
             LogContainer.cpp)
            
qt4_wrap_cpp(PROJ_MOC_FILES MainWindow.h PointCloudViewer.h
            ImageContainer.h LogContainer.h 
            ExecutionControl.h StatisticsContainer.h
            KWidgetBase.h LogicBridge.h
            RobXControl.h RobXControlDialog.h
            OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt4_automoc(${GUI_SRCS})

add_library(ksrobot_gui SHARED ${GUI_SRCS} ${PROJ_MOC_FILES})

target_link_libraries(ksrobot_gui ksrobot_common ksrobot_interfaces ${GLOBAL_LIBRARIES} ${QT_QTCORE_LIBRARY} 
                       ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} ${VTK_LIBRARIES} QVTK)
 
