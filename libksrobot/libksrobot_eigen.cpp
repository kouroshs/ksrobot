#include <Eigen/Geometry>
#include <sstream>
#include <boost/python.hpp>

using namespace Eigen;


class DummyScopeClass_Eigen
{
};

template<class X>
void SetterQX(Eigen::Quaternion<X>&q, const X& value)
{
    q.x() = value;
}
template<class X>
void SetterQY(Eigen::Quaternion<X>&q, const X& value)
{
    q.y() = value;
}
template<class X>
void SetterQZ(Eigen::Quaternion<X>&q, const X& value)
{
    q.z() = value;
}
template<class X>
void SetterQW(Eigen::Quaternion<X>&q, const X& value)
{
    q.w() = value;
}

template<class X, int Index>
X GetQ(const Eigen::Quaternion<X>& q)
{
    switch(Index)
    {
        case 0:
            return q.w();
        case 1:
            return q.x();
        case 2:
            return q.y();
        case 3:
            return q.z();
    }
}

template<class X, int Index>
void SetQ(Eigen::Quaternion<X>& q, const X& x)
{
    switch(Index)
    {
        case 0:
            q.w() = x;
            return;
        case 1:
            q.x() = x;
            return;
        case 2:
            q.y()= x;
            return;
        case 3:
            q.z() = x;
            return;
    }
}

Quaternionf CastQToFloat(const Quaterniond& q)
{
    return q.cast<float>();
}

Quaterniond CastQToDouble(const Quaternionf& q)
{
    return q.cast<double>();
}

template<class X>
std::string QToString(const Quaternion<X>& q)
{
    std::stringstream ss;
    ss << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", "<< q.z() << ")";
    return ss.str();
}

template<class MatrixType>
std::string MatrixToString(const MatrixType& m)
{
    std::stringstream ss;
    ss << m;
    return ss.str();
}


template<class X>
void ExportMatrix3(const char* name)
{
    using namespace boost::python;
    using namespace Eigen;

    typedef typename Eigen::Matrix<X, 3, 3> Mtx;
    
    
    class_<Mtx>(name, init<>())
        .def_readonly("IsDouble", boost::is_same<X, double>::value)
        .def("fill", &Mtx::fill)
        .def(self * Mtx())
        .def(self * X())
        .def(self / X())
        .def(self - Mtx())
        .def(self + Mtx())
        .def(self == Mtx())
        .def("__str__", make_function(MatrixToString<Mtx>))
    ;
}


template<class X>
void ExportVector3(const char* name)
{
    Eigen::Vector3d a;
}

template <class X, class Y, int D>
X GetV(const Y& v)
{
    return v[D];
}

template<class X, class Y, int D>
void SetV(Y& v, const X& x)
{
    v[D] = x;
}

template<class X, int D>
std::string VectorToString(const X& v)
{
    std::stringstream ss;
    ss << "(";
    for(int i = 0; i < D - 1; i++)
        ss << v[i] << ", ";
    ss << v[D - 1] << ")";
    return ss.str();
}

// template<class X>
// std::string EigenTypeToString(const X& x)
// {
//     std::stringstream ss;
//     ss << x;
//     return ss.str();
// }


std::string IsometrydToString(const Isometry3d& x)
{
    std::stringstream ss;
    ss << x.matrix();
    return ss.str();
}

std::string IsometryfToString(const Isometry3f& x)
{
    std::stringstream ss;
    ss << x.matrix();
    return ss.str();
}

Isometry3d IsometryCastToDouble(const Isometry3f& iso)
{
    return iso.cast<double>();
}

Isometry3f IsometryCastToFloat(const Isometry3d& iso)
{
    return iso.cast<float>();
}

static Quaterniond qd_identity(Quaterniond::Identity());
static Quaternionf qf_identity(Quaternionf::Identity());
static Isometry3d iso_identity_d(Isometry3d::Identity());
static Isometry3f iso_identity_f(Isometry3f::Identity());

void ExportEigen()
{
    using namespace boost::python;
    using namespace Eigen;
    scope namespace_scope = class_<DummyScopeClass_Eigen>("eigen");
    
    
    
    class_<Quaterniond>("Quaterniond", init<>())
        .def(init<double,double,double,double>())
        .def_readonly("IsDouble", true)
        .def_readonly("Identity", qd_identity)
        .add_property("w", make_function(GetQ<double, 0>), make_function(SetQ<double, 0>))
        .add_property("x", make_function(GetQ<double, 1>), make_function(SetQ<double, 1>))
        .add_property("y", make_function(GetQ<double, 2>), make_function(SetQ<double, 2>))
        .add_property("z", make_function(GetQ<double, 3>), make_function(SetQ<double, 3>))
        .def("conjugate", &Quaterniond::conjugate)
        .def("dot", &Quaterniond::dot<Quaterniond>)
        .def("ToFloatQuaternion", make_function(CastQToFloat))
        .def("inverse", &Quaterniond::inverse)
        .def("norm", &Quaterniond::norm)
        .def("normalize", &Quaterniond::normalize)
        .def("normalized", &Quaterniond::normalized)
        .def("squaredNorm", &Quaterniond::squaredNorm)
        .def("toRotationMatrix", &Quaterniond::toRotationMatrix)
        
        .def(self * Quaterniond())
        .def(self * Matrix3d())
        .def(self * Isometry3d())
        
        .def("__str__", make_function(QToString<double>))
    ;

    class_<Quaternionf>("Quaternionf", init<>())
        .def(init<const Matrix3f&>())
        .def(init<float,float,float,float>())
        .def_readonly("IsDouble", false)
        .def_readonly("Identity", qf_identity)
        .add_property("w", make_function(GetQ<float, 0>), make_function(SetQ<float, 0>))
        .add_property("x", make_function(GetQ<float, 1>), make_function(SetQ<float, 1>))
        .add_property("y", make_function(GetQ<float, 2>), make_function(SetQ<float, 2>))
        .add_property("z", make_function(GetQ<float, 3>), make_function(SetQ<float, 3>))
        .def("conjugate", &Quaternionf::conjugate)
        .def("dot", &Quaternionf::dot<Quaternionf>)
        .def("ToDoubleQuaternion", make_function(CastQToDouble))
        .def("inverse", &Quaternionf::inverse)
        .def("norm", &Quaternionf::norm)
        .def("normalize", &Quaternionf::normalize)
        .def("normalized", &Quaternionf::normalized)
        .def("squaredNorm", &Quaternionf::squaredNorm)
        .def("toRotationMatrix", &Quaternionf::toRotationMatrix)
        
        .def(self * Quaternionf())
        .def(self * Matrix3f())
        .def(self * Isometry3f())
        
        .def("__str__", make_function(QToString<float>))
    ;
    
    ExportMatrix3<double>("Matrix3d");
    ExportMatrix3<float>("Matrix3f");
    
    class_<Vector3d>("Vector3d", init<>())
        .def(init<double, double, double>())
        .add_property("x", make_function(GetV<double, Vector3d, 0>), make_function(SetV<double, Vector3d, 0>))
        .add_property("y", make_function(GetV<double, Vector3d, 1>), make_function(SetV<double, Vector3d, 1>))
        .add_property("z", make_function(GetV<double, Vector3d, 2>), make_function(SetV<double, Vector3d, 2>))
        
        .def(self + Vector3d())
        .def(self - Vector3d())
        .def(self * double())
        .def(self / double())
        
        .def("__str__", make_function(VectorToString<Vector3d, 3>))
    ;

    class_<Vector3f>("Vector3f", init<>())
        .def(init<float, float, float>())
        .add_property("x", make_function(GetV<float, Vector3d, 0>), make_function(SetV<float, Vector3d, 0>))
        .add_property("y", make_function(GetV<float, Vector3d, 1>), make_function(SetV<float, Vector3d, 1>))
        .add_property("z", make_function(GetV<float, Vector3d, 2>), make_function(SetV<float, Vector3d, 2>))
        
        .def(self + Vector3f())
        .def(self - Vector3f())
        .def(self * float())
        .def(self / float())
        
        .def("__str__", make_function(VectorToString<Vector3f, 3>))
    ;

    class_<Vector4d>("Vector4d", init<>())
        .def(init<double, double, double, double>())
        .add_property("x", make_function(GetV<double, Vector3d, 0>), make_function(SetV<double, Vector3d, 0>))
        .add_property("y", make_function(GetV<double, Vector3d, 1>), make_function(SetV<double, Vector3d, 1>))
        .add_property("z", make_function(GetV<double, Vector3d, 2>), make_function(SetV<double, Vector3d, 2>))
        .add_property("w", make_function(GetV<double, Vector3d, 3>), make_function(SetV<double, Vector3d, 3>))
        
        .def(self + Vector4d())
        .def(self - Vector4d())
        .def(self * double())
        .def(self / double())
        
        .def("__str__", make_function(VectorToString<Vector4d, 4>))
    ;

    class_<Vector4f>("Vector4f", init<>())
        .def(init<float, float, float, float>())
        .add_property("x", make_function(GetV<float, Vector3f, 0>), make_function(SetV<float, Vector3f, 0>))
        .add_property("y", make_function(GetV<float, Vector3f, 1>), make_function(SetV<float, Vector3f, 1>))
        .add_property("z", make_function(GetV<float, Vector3f, 2>), make_function(SetV<float, Vector3f, 2>))
        .add_property("w", make_function(GetV<float, Vector3f, 3>), make_function(SetV<float, Vector3f, 3>))
        
        .def(self + Vector4f())
        .def(self - Vector4f())
        .def(self * float())
        .def(self / float())
        
        .def("__str__", make_function(VectorToString<Vector4f, 4>))
    ;
    
    
    class_<Isometry3d>("Isometry3d", init<>())
        .def_readonly("IsDouble", true)
        .def("rotation", &Isometry3d::rotation)
        //.def("translation", &Isometry3d::translation)
        .def_readonly("Identity", iso_identity_d)
        .def("CastToFloat", make_function(IsometryCastToFloat))
        .def("__str__", make_function(IsometrydToString))
    ;
    class_<Isometry3f>("Isometry3f", init<>())
        .def_readonly("IsDouble", false)
        .def("rotation", &Isometry3f::rotation)
        //.def("translation", &Isometry3f::translation)
        .def("CastToDouble", make_function(IsometryCastToDouble))
        .def_readonly("Identity", iso_identity_f)
        .def("__str__", make_function(IsometryfToString))
    ;
}