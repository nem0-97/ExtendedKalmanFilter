#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    //measurement matrix - laser
    H_laser_ << 1,0,0,0,
    0,1,0,0;
    
    //Set Hj later using tools.CalculateJacobian(ekf_.x_) right befor radar updates?Is Hj even necessary as a property here?
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        //Initial state covariance
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
        
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;//Is this needed?
        
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {//Drop the velocity measure here? Assume 0,0 as initial velocity and just use position measured by radar?
            // set the state with the initial location and velocities
            float cosPhi=cos(measurement_pack.raw_measurements_[1]);
            float sinPhi=sin(measurement_pack.raw_measurements_[1]);
            ekf_.x_ << measurement_pack.raw_measurements_[0]*cosPhi,measurement_pack.raw_measurements_[0]*sinPhi,0,0;
            //measurement_pack.raw_measurements_[2]*cosPhi,measurement_pack.raw_measurements_[2]*sinPhi; Can't do this?
            
            //calculate and set initial state covariance(high covariance for those not measured velocities)
            ekf_.P_<<  .09, 0, 0, 0,
            0, .0009, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // set the state with the initial location and zero velocity
            ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;
            //set initial state covariance as measurement covariance(high covariance for those not measured velocities)
            ekf_.P_<<  .0225, 0, 0, 0,
            0, .0225, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
        }
        
        //set initial start time of first measurement
        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /**
     * Prediction(Uses F and Q)
     */
    
    //Get delta time in seconds and update previous timestamp to this one for the next update
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    //Update F matrix with dt
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_<< 1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    //Calculate covariance matrix Q
    float noise_ax=9;
    float noise_ay=9;
    float dt2=dt*dt;
    float dt3=dt2*dt;
    float dt4=dt3*dt;
    
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
    0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
    dt3/2*noise_ax, 0, dt2*noise_ax, 0,
    0, dt3/2*noise_ay, 0, dt2*noise_ay;
    
    ekf_.Predict();
    
    /**
     * Update(Uses H and R)
     */
    
    //Based on sensor type set ekf_'s H and R matrices to proper values and call appropriate update to update state and state covariance
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        Hj_= tools.CalculateJacobian(ekf_.x_);
        ekf_.H_=Hj_;//3x4
        ekf_.R_=R_radar_;//3x3
        
        //initialize z
        VectorXd z=VectorXd(3);
        z<<measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],measurement_pack.raw_measurements_[2];
        
        ekf_.UpdateEKF(z);
    } else {
        ekf_.H_=H_laser_;//2x4
        ekf_.R_=R_laser_;//2x2
        
        //initialize z
        VectorXd z=VectorXd(2);
        z<<measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1];
        ekf_.Update(z);
    }
    
    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
