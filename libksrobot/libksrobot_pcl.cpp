#include <Eigen/Geometry>
#include <boost/python.hpp>

class DummyScopeClass_PCL
{
};

void ExportPCL()
{
    using namespace boost::python;
    scope namespace_scope = class_<DummyScopeClass_PCL>("pcl");
    
    
}