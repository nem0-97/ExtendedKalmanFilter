#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in){
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {//Prediction step uses linear model, no need to use Fj instead of F or F(x')
    // KF Prediction step
    x_=F_*x_;//U is 0 so leave it out
    P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {//H*x turns it into 2d vector(px,py) to match z
    //calculate y s and k
    MatrixXd Ht=H_.transpose();
    MatrixXd y=z-H_*x_;
    MatrixXd S=H_*P_*Ht+R_;
    MatrixXd K=P_*Ht*S.inverse();
    
    // new state
    x_=x_+(K*y);
    P_=(MatrixXd::Identity(x_.size(), x_.size())-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {//same as Update but convert x from 4d vector(px,py,vx,vy) to 3d vector(rho,phi,rho_dot) to match z
    //calculate h(x')
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float r=sqrt(px*px+py*py);
    
    VectorXd Hx=VectorXd(3);//atan2 returns between -pi and pi
    Hx<< r,atan2(py,px),(px*vx+py*vy)/r;//check if r is 0?(shouldn't be)
    
    //calculate y s and k
    MatrixXd Ht=H_.transpose();//4x3
    MatrixXd y=z-Hx;
    //normalize y(1) phi between -pi and pi(should it be inclusive of both?)
    while(y(1)<=-M_PI){
        y(1)+=2*M_PI;
    }
    while(y(1)>M_PI){
        y(1)-=2*M_PI;
    }
    
    MatrixXd S=H_*P_*Ht+R_;
    MatrixXd K=P_*Ht*S.inverse();
    
    // new state
    x_=x_+(K*y);
    P_=(MatrixXd::Identity(x_.size(), x_.size())-K*H_)*P_;
}
