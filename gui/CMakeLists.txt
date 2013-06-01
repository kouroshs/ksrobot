set(GUI_SRCS LogicBridge.cpp KWidgetBase.cpp StatisticsContainer.cpp ExecutionControl.cpp 
            Utils.cpp MainWindow.cpp 
            ConfigDialog.cpp ImageContainer.cpp 
            LogContainer.cpp)
            
qt4_wrap_cpp(PROJ_MOC_FILES MainWindow.h ConfigDialog.h 
            ImageContainer.h LogContainer.h 
            ExecutionControl.h StatisticsContainer.h
            KWidgetBase.h LogicBridge.h
            OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt4_automoc(${GUI_SRCS})

add_library(gui STATIC ${GUI_SRCS} ${PROJ_MOC_FILES})
target_link_libraries(gui ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
#install(TARGETS gui RUNTIME DESTINATION bin)
 
