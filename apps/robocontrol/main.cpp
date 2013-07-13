#include <iostream>
#include <roboctrl/RobXControlDialog.h>
#include <QApplication>

#include <Eigen/Geometry>

template<size_t dims>
class UncertainValue
{
public:
    static const size_t Dim = dims;
    
    typedef Eigen::Matrix<double, Dim, 1>       Vector;
    typedef Eigen::Matrix<double, Dim, Dim>     Matrix;
};

template<int StateDim, int ObservationDim>
class KalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef UncertainValue<StateDim>                    State;
    typedef UncertainValue<ObservationDim>              Observation;
    
    
    void Update()
    {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }
    
    void Predict(const typename Observation::Vector& z)
    {
        typename Observation::Vector y = z - H * x;
        typename Observation::Matrix S = H * P * H.transpose() + R;
        typename Eigen::Matrix<double, StateDim, ObservationDim> K;
        K = P * H.transpose() * S.inverse();
        x = x + K * y;
        P = (State::Matrix::Identity() - K * H) * P;
    }
    
    public: // public data members
        typename State::Matrix                       Q;
        typename State::Matrix                       P;
        typename State::Matrix                       F;
        
        typename Eigen::Matrix<double, ObservationDim, StateDim>   H;
        
        typename Observation::Matrix                 R;
        
        typename State::Vector                       x;
};


int main(int argc, char** argv)
{
    typedef KalmanFilter<6, 3> KF;
    KF kf;
    kf.F.setIdentity();
    kf.H.setIdentity();
    kf.P.setIdentity();
    kf.P *= 0.1;
    kf.Q.setIdentity();
    kf.R.setIdentity();
    kf.R *= 0.1;
    kf.x << 1 , 0.5, 0, 0, 0, 0;
    
    KF::Observation::Vector z;
    z << 1, 0.4, 0;
    
    kf.Update();
    kf.Predict(z);
    
    std::cout << kf.x << std::endl;
    
    return 0;
//     QApplication app(argc, argv);
//     KSRobot::roboctrl::RobXControlDialog dlg;
//     dlg.show();
//     
//     return app.exec();
}

