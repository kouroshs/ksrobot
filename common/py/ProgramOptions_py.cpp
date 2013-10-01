#include <common/ProgramOptions.h>
#include <boost/python.hpp>

using namespace KSRobot::common;
using namespace boost::python;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIntOverloads, GetInt, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetDoubleOverloads, GetDouble, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetStringOverloads, GetString, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetBoolOverloads, GetBool, 1, 2)

template<typename T>
struct optional_ : private boost::noncopyable {
    struct conversion : public boost::python::converter::expected_from_python_type<T> {
        static PyObject* convert(boost::optional<T> const& value)
        
        {
            using namespace boost::python;
            return incref((value ? object(*value) : object()).ptr());
        }
    };
    
    static void* convertible(PyObject *obj) {
        using namespace boost::python;
        return obj == Py_None || extract<T>(obj).check() ? obj : NULL;
    }
    
    static void constructor(
        PyObject *obj,
        boost::python::converter::rvalue_from_python_stage1_data *data
    ) {
        using namespace boost::python;
        void *const storage =
        reinterpret_cast<
        converter::rvalue_from_python_storage<boost::optional<T> >*
        >(data)->storage.bytes;
        if(obj == Py_None) {
            new (storage) boost::optional<T>();
        } else {
            new (storage) boost::optional<T>(extract<T>(obj));
        }
        data->convertible = storage;
    }
    
    explicit optional_() {
        using namespace boost::python;
        if(!extract<boost::optional<T> >(object()).check()) {
            to_python_converter<boost::optional<T>, conversion, true>();
            converter::registry::push_back(
                &convertible,
                &constructor,
                type_id<boost::optional<T> >(),
                                           &conversion::get_pytype
            );
        }
    }
    
};


CLASS_PYEXPORT_FN(ProgramOptions)
{
    optional_<bool>();
    optional_<int>();
    optional_<double>();
    optional_<std::string>();
    
    class_<ProgramOptions, ProgramOptions::Ptr>("ProgramOptions", init<>())
    .def("LoadFromFile", &ProgramOptions::LoadFromFile)
    .def("SaveToFile", &ProgramOptions::SaveToFile)
    .def("NodeExists", &ProgramOptions::NodeExists)
    .def("StartNode", &ProgramOptions::StartNode)
    .def("GetInt", &ProgramOptions::GetInt, GetIntOverloads())
    .def("GetBool", &ProgramOptions::GetBool, GetBoolOverloads())
    .def("GetDouble", &ProgramOptions::GetDouble, GetDoubleOverloads())
    .def("GetString", &ProgramOptions::GetString, GetStringOverloads())
    .def("PutInt", &ProgramOptions::PutInt)
    .def("PutBool", &ProgramOptions::PutBool)
    .def("PutDouble", &ProgramOptions::PutDouble)
    .def("PutString", &ProgramOptions::PutString)
    ;
}