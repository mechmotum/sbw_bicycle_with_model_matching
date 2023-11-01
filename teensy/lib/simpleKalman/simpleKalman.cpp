#include "simpleKalman.h"

SimpleKalman::SimpleKalman(
        Matrix<float,2,2>& F,
        Matrix<float,2,1>& B,
        Matrix<float,1,2>& H,
        Matrix<float,2,2>& Q,
        Matrix<float,1,1>& R,
        Matrix<float,2,2>& P_post)
: F(F), B(B), H(H), Q(Q), R(R), P_post(P_post)
{
I.setIdentity();
P_prio.setZero();
x_prio.setZero();
}

void SimpleKalman::init(){
    x_post.setZero();
    t = 0;
}

void SimpleKalman::init(Matrix<float,2,1>& x0, double t0){
    x_post = x0;
    t = t0;
}

void SimpleKalman::next_step(Matrix<float,1,1>& u, Matrix<float,1,1>& z, double dt){
    predict_step(u, dt);
    update_step(z);
}

void SimpleKalman::predict_step(Matrix<float,1,1>& u, double dt){
    x_prio = F*x_post + B*u;
    P_prio = F*P_post*F.transpose() + Q;
    t += dt;
}

void SimpleKalman::update_step(Matrix<float,1,1>& z){
    K = (P_prio*H.transpose()) * (H*P_prio*H.transpose() + R).cwiseInverse(); //Only works since (H*P_prio*H.transpose() + R) is 1x1 (scalar) otherwise a Ax = b type of solver has to be used;
    x_post = x_prio + K*(z - H*x_prio);
    P_post = (I - K*H)*P_prio;
}

float SimpleKalman::phi(){
    return this->x_post(1,1);
}

float SimpleKalman::bias(){
    return this->x_post(2,1);
}

double SimpleKalman::time(){
    return this->t;
}
