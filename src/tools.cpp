#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    if(estimations.size()==0){
        cout<<"CalculateRMSE () - Error - Should Be > Zero Estimations"<<endl;
    }else if(estimations.size()!=ground_truth.size()){
        cout<<"CalculateRMSE () - Error - Should Be Same Number Estimations As Ground Truths"<<endl;
    }else{
        //accumulate squared residuals
        for (int i=0; i < estimations.size(); ++i) {
            VectorXd res=(estimations[i]-ground_truth[i]);
            res=res.array()*res.array();
            rmse+=res;
        }
        //root of the mean of squares of the residuals(rmse)
        rmse=(rmse/estimations.size()).array().sqrt();
    }
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //calculate formula component that would have to be 0 to get divide by 0 error
    float r2=px*px+py*py;
    
    //check proper components to avoid division by zero
    if(r2==0){//r^2and r^3 =0 iff r=0 so only need to check one
        cout<<"CalculateJacobian () - Error - Division by Zero"<<endl;
    }else{
        //calculate rest of formula components only once
        float r = sqrt(r2);
        float r3=r2*r;
        float vxpy=vx*py;
        float vypx=vy*px;
        //compute the Jacobian matrix
        Hj<<px/r,py/r,0,0,
        -py/r2,px/r2,0,0,
        py*(vxpy-vypx)/r3,px*(vypx-vxpy)/r3,px/r,py/r;
    }
    return Hj;
}
