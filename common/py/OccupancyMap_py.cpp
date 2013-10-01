#include <common/OccupancyMap.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace KSRobot::common;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(OccupancyMapExtractFrontiers, ExtractFrontiers, 1, 2)

void OccupancyConvertToNumpy(KSRobot::common::OccupancyMap::Ptr ocmap)
{
    (void)ocmap;
    //TODO: IMPLEMENT THIS
}

CLASS_PYEXPORT_FN(OccupancyMap)
{
    class_<std::vector<OccupancyMap::MapPointDataType> >("OccupancyMapArray")
        .def(vector_indexing_suite<std::vector<OccupancyMap::MapPointDataType> >())
    ;
    
    scope occmap_class_scope =
    class_<OccupancyMap, boost::shared_ptr<OccupancyMap> >("OccupancyMap", init<>())
        .def(init<size_t, size_t>())
        .def("LoadFromFile", &OccupancyMap::LoadFromFile)
        .def("SaveToFile", &OccupancyMap::SaveToFile)
        .def("GetSize", &OccupancyMap::GetSize)
        .def("Index", &OccupancyMap::Index)
        .def("ConvertToNumpy", make_function(OccupancyConvertToNumpy))
        .def("IsValidIndex", &OccupancyMap::IsValidIndex)
        .def("IsValidPosition", &OccupancyMap::IsValidPosition)
        .def("AddPointToROI", &OccupancyMap::AddPointToROI)
        .def("ExtractFrontiers", &OccupancyMap::ExtractFrontiers, OccupancyMapExtractFrontiers())
        .def_readwrite("Width", &OccupancyMap::Width)
        .def_readwrite("Height", &OccupancyMap::Height)
        .def_readwrite("OccupiedCellsCount", &OccupancyMap::OccupiedCellsCount)
        .def_readwrite("FreeCellsCount", &OccupancyMap::FreeCellsCount)
        .def_readwrite("UnknownCellsCount", &OccupancyMap::UnknownCellsCount)
        .def_readwrite("CenterX", &OccupancyMap::CenterX)
        .def_readwrite("CenterY", &OccupancyMap::CenterY)
        .def_readwrite("Resolution", &OccupancyMap::Resolution)
        .def_readwrite("ROI", &OccupancyMap::ROI)
        .def_readwrite("Data", &OccupancyMap::Data)
    ;
    
    enum_<OccupancyMap::CellValue>("CellValue")
        .value("UnknownCell", OccupancyMap::UnknownCell)
        .value("OccupiedCell", OccupancyMap::OccupiedCell)
        .value("FreeCell", OccupancyMap::FreeCell)
        .value("FrontierCell", OccupancyMap::FrontierCell)
        .value("FrontierStartID", OccupancyMap::FrontierStartID)
        .value("FrontierEndID", OccupancyMap::FrontierEndID)
    ;
    
    class_<OccupancyMap::Rect>("Rect", init<>())
        .def_readwrite("Top", &OccupancyMap::Rect::Top)
        .def_readwrite("Left", &OccupancyMap::Rect::Left)
        .def_readwrite("Right", &OccupancyMap::Rect::Right)
        .def_readwrite("Buttom", &OccupancyMap::Rect::Buttom)
    ;
    
    //TODO: Needs definition of Eigen::Vector2f and Eigen::Matrix2f
    class_<OccupancyMap::Frontier>("Frontier", init<>())
        .def_readwrite("Position", &OccupancyMap::Frontier::Position)
        .def_readwrite("Variance", &OccupancyMap::Frontier::Variance)
        .def_readwrite("Rotation", &OccupancyMap::Frontier::Rotation)
        .def_readwrite("Size", &OccupancyMap::Frontier::Size)
        .def_readwrite("ID", &OccupancyMap::Frontier::ID)
    ;
    
    class_<std::vector<OccupancyMap::Frontier> >("FrontierArray")
        .def(vector_indexing_suite<std::vector<OccupancyMap::Frontier> >())
    ;
}

