/*
This class definition is specifically made for a kalman implementation
as described in Sanjurjo e.a. 2019 "Roll angle estimator based on angular 
rate measurements for bicycles". 
Dimensions, and other generalization have been ignored, for the sake
of simplicity and speed.

Altough this class can serve as a basis for a general Kalman filter,
the use of a well reviewed library is recommended instead.
*/

/*
Dimensions:
x: 2x1, u: 1x1, z: 1x1
F: 2x2, B: 2x1, H: 1x2 
Q: 2x2, R: 1x1, P: 2x2
*/

#ifndef SIMPLE_KALMAN_
#define SIMPLE_KALMAN_

#include "eigen.h"
using namespace Eigen;

class SimpleKalman{
    public:
    SimpleKalman(Matrix<float,2,2>& F, 
                 Matrix<float,2,1>& B, 
                 Matrix<float,1,2>& H, 
                 Matrix<float,2,2>& Q, 
                 Matrix<float,1,1>& R,
                 Matrix<float,2,2>& P_post);

    void init();
    void init(const Matrix<float,2,1>& x0, const double t0);

    void predict_step(Matrix<float,1,1>& u, double dt);
    void update_step(Matrix<float,1,1>& z);
    void next_step(Matrix<float,1,1>& u, Matrix<float,1,1>& z, double dt);

    // implementation specific functions
    float phi(); //return the estimated roll angle
    float bias(); //return the estimated gyroscope bias
    double time();

    //--[Setters
    void set_F(Matrix<float,2,2>& F){m_F=F;}
    void set_B(Matrix<float,2,1>& B){m_B=B;}

    private:
    //--[Time
    double t; //time

    //--[Matrices
    // sysem matrices
    Matrix<float,2,2> m_F; // State transition model
    Matrix<float,2,1> m_B; // Control input model 
    Matrix<float,1,2> H; // Observation model
    Matrix<float,2,2> Q; // Process noise covariance
    Matrix<float,1,1> R; // Measurement noise covariance

    //Helping matrices
    Matrix<float,2,2> P_post; // P_k|k updated csovariance matrix
    Matrix<float,2,2> P_prio; // P_k+1|k predicted covariance matrix
    Matrix<float,2,1> K; // Kalman gain
    Matrix<float,2,2> I; // Identity

    //state
    Matrix<float,2,1> x_post; // x_k|k updated state estimate
    Matrix<float,2,1> x_prio; // x_k+1|k predicted state estimate
};

#endif //SIMPLE_KALMAN_

//TODO: Write P_post and P_prio to the same variable P. so less space needed.
//TODO: Use an `initialized` bool to force the Kalman filter to be initialized before it can be used