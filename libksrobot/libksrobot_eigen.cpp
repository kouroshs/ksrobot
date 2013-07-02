#include <Eigen/Geometry>
#include <sstream>
#include <boost/python.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <stdio.h>

using namespace Eigen;


class DummyScopeClass_Eigen
{
};


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


void ExportMatrix3f(const char* name)
{
    using namespace boost::python;
    using namespace Eigen;

    typedef typename Eigen::Matrix3f Mtx;
    
    
    class_<Mtx>(name, init<>())
        .def("fill", &Mtx::fill)
        .def(self * Mtx())
        .def(self * float())
        .def(self / float())
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

Vector3d Isometry3d_translation3(const Isometry3d& iso)
{
    return iso.translation();
}

Vector4d Isometry3d_translation4(const Isometry3d& iso)
{
    Vector3d v = iso.translation();
    return Vector4d(v[0], v[1], v[2], 1);
}

struct PyAngleAxisf
{
    void Register()
    {
        using namespace boost::python;
        using namespace Eigen;
        class_<AngleAxisf>("AngleAxisf", init<>())
            .def(init<float, const Vector3f&>())
            .def(init<const Quaternionf&>())
            .def(init<const Matrix3f&>())
            .add_property("angle", make_function(PyAngleAxisf::GetAngle), make_function(PyAngleAxisf::SetAngle))
            .add_property("axis", make_function(PyAngleAxisf::GetAxis), make_function(PyAngleAxisf::SetAxis))
            .def("inverse", &AngleAxisf::inverse)
            .def("toRotationMatrix", &AngleAxisf::toRotationMatrix)
        ;
        
    }
    
    static float GetAngle(const AngleAxisf& aa)
    {
        return aa.angle();
    }
    
    static void SetAngle(AngleAxisf& aa, const float& val)
    {
        aa.angle() = val;
    }
    
    static Vector3f GetAxis(const AngleAxisf& aa)
    {
        return aa.axis();
    }
    
    static void SetAxis(AngleAxisf& aa, const Vector3f& v)
    {
        aa.axis() = v;
    }
};

struct PyQuaternionf
{
    Quaternionf qf_identity;
    
    PyQuaternionf()
    {
        qf_identity.setIdentity();
    }

    void Register()
    {
        using namespace boost::python;
        using namespace Eigen;
        class_<Quaternionf>("Quaternionf", init<>())
            .def(init<const Matrix3f&>())
            .def(init<float,float,float,float>())
            .def_readonly("Identity", qf_identity)
            .add_property("w", make_function(&PyQuaternionf::GetQ<0>), make_function(&PyQuaternionf::SetQ<0>))
            .add_property("x", make_function(&PyQuaternionf::GetQ<1>), make_function(&PyQuaternionf::SetQ<1>))
            .add_property("y", make_function(&PyQuaternionf::GetQ<2>), make_function(&PyQuaternionf::SetQ<2>))
            .add_property("z", make_function(&PyQuaternionf::GetQ<3>), make_function(&PyQuaternionf::SetQ<3>))
            .def("conjugate", &Quaternionf::conjugate)
            .def("dot", &Quaternionf::dot<Quaternionf>)

            .def("inverse", &Quaternionf::inverse)
            .def("norm", &Quaternionf::norm)
            .def("normalize", &Quaternionf::normalize)
            .def("normalized", &Quaternionf::normalized)
            .def("squaredNorm", &Quaternionf::squaredNorm)
            .def("toRotationMatrix", &Quaternionf::toRotationMatrix)
            
            .def(self * Quaternionf())
            .def(self * Matrix3f())
            .def(self * Isometry3f())
            .def("__str__", make_function(&PyQuaternionf::ToString))
            .def("FromString", make_function(&PyQuaternionf::FromString))
        ;
    }

    static void FromString(Quaternionf& q, const std::string& str)
    {
        float w,x,y,z;
        if( sscanf(str.c_str(), "(%f, %f, %f, %f)", &w, &x, &y, &z) != 4 )
            throw std::runtime_error("Invalid quaternion string <" + str + ">.");
        q = Quaternionf(w, x, y, z);
    }
    
    static std::string ToString(const Quaternionf& q)
    {
        std::stringstream ss;
        ss << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", "<< q.z() << ")";
        return ss.str();
    }
    
    template<int Index>
    static float GetQ(const Eigen::Quaternionf& q)
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
    
    template<int Index>
    static void SetQ(Eigen::Quaternionf& q, const float& x)
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
};

static Isometry3d iso_identity_d(Isometry3d::Identity());
static Isometry3f iso_identity_f(Isometry3f::Identity());

static PyAngleAxisf angle_axis;
static PyQuaternionf quaternion;

static Vector3f unit_x = Vector3f::UnitX(), unit_y = Vector3f::UnitY(), unit_z = Vector3f::UnitZ();

void ExportEigen()
{
    using namespace boost::python;
    using namespace Eigen;
    scope namespace_scope = class_<DummyScopeClass_Eigen>("eigen");

    angle_axis.Register();
    quaternion.Register();
    
    class_<Vector3f>("Vector3f", init<>())
        .def(init<float, float, float>())
        .add_property("x", make_function(GetV<float, Vector3d, 0>), make_function(SetV<float, Vector3d, 0>))
        .add_property("y", make_function(GetV<float, Vector3d, 1>), make_function(SetV<float, Vector3d, 1>))
        .add_property("z", make_function(GetV<float, Vector3d, 2>), make_function(SetV<float, Vector3d, 2>))
        
        .def_readonly("UnitX", unit_x)
        .def_readonly("UnitY", unit_y)
        .def_readonly("UnitZ", unit_z)
        
        .def(self + Vector3f())
        .def(self - Vector3f())
        .def(self * float())
        .def(self / float())
        
        .def("__str__", make_function(VectorToString<Vector3f, 3>))
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


    
    
    ExportMatrix3f("Matrix3f");
    
    class_<Isometry3f>("Isometry3f", init<>())
        .def_readonly("IsDouble", false)
        .def("rotation", &Isometry3f::rotation)
        //.def("translation", &Isometry3f::translation)
        .def("CastToDouble", make_function(IsometryCastToDouble))
        .def_readonly("Identity", iso_identity_f)
        .def("__str__", make_function(IsometryfToString))
    ;
}