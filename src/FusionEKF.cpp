#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define DBGMSG( os, msg ) {}

#define NDBGMSG( os, msg ) \
  (os) << msg << std::endl

/*
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

    /**
    TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */

    //Create the ekf_object, assume that it is using lidar first

    H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    Hj_ << 1, 1, 0, 0,
          1, 1, 0, 0,
          1, 1, 1, 1;

    noise_ax = 9;
    noise_ay = 9;
    initial_conv = 1000;

    Eigen::MatrixXd P_, Q_, F_;

    F_ = MatrixXd(4, 4);
    update_F(F_, 0);

    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, initial_conv, 0,
          0, 0, 0, initial_conv;

    Q_ = MatrixXd(4, 4);
    update_Q(Q_, 0);

    VectorXd x_;
    x_ = VectorXd(4);

    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
    *  Initialization
    ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */


        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            DBGMSG(cout, "EKF initialized with RADAR: " );

            float rho, phi, rho_dot, px, py;
            rho = measurement_pack.raw_measurements_[0];
            phi = measurement_pack.raw_measurements_[1];
            rho_dot = measurement_pack.raw_measurements_[2];

            px = rho * cos(phi);
            py = rho * sin(phi);

            ekf_.x_ << px, py, rho_dot * cos(phi), rho_dot * sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            DBGMSG(cout, "EKF initialized with LIDAR: " );

            float px, py;
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];

            ekf_.x_ << px, py, 0, 0;
        }

        // done initializing, no need to predict or update

        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;

        DBGMSG(cout, "x_ (initialize) = " << ekf_.x_ );
        DBGMSG(cout, "P_ (initialize) = " << ekf_.P_ );

        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/

    /**
    TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
    */
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    update_F(ekf_.F_, dt);
    update_Q(ekf_.Q_, dt);

    ekf_.Predict();

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    /**
    TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        if(!isinf(Hj_(0, 0))) {
            DBGMSG(cout, "RADAR" );
            ekf_.H_ = Hj_;
            ekf_.R_ = R_radar_;
            ekf_.h_ = &FusionEKF::getZPredictRadar;
            ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        }
        else
            DBGMSG(cout, "RADAR skipped" );

    } else {
        DBGMSG(cout, "LIDAR" );
        // Laser updatesmeas_package.sensor_type_ = MeasurementPackage::LASER;
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    DBGMSG(cout, "x_ = " << ekf_.x_ );
    DBGMSG(cout, "P_ = " << ekf_.P_ );
}

void FusionEKF::update_F(Eigen::MatrixXd& F_, float dt) {
    F_(0, 2) = dt;
    F_(1, 3) = dt;

    F_(0, 0) = F_(1, 1) = F_(2, 2) = F_(3, 3)= 1;
    F_(0, 1) = F_(0, 3) = 0;
    F_(1, 0) = F_(1, 2) = 0;
    F_(2, 0) = F_(2, 1) = F_(2, 3) = 0;
    F_(3, 0) = F_(3, 1) = F_(3, 2) = 0;
}

void FusionEKF::update_Q(Eigen::MatrixXd& Q_, float dt){
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    Q_(0, 0) = dt_4/4*noise_ax;
    Q_(0, 2) = dt_3/2*noise_ax;
    Q_(1, 1) = dt_4/4*noise_ay;
    Q_(1, 3) = dt_3/2*noise_ay;
    Q_(2, 0) = dt_3/2*noise_ax;
    Q_(2, 2) = dt_2*noise_ax;
    Q_(3, 1) = dt_3/2*noise_ay;
    Q_(3, 3) = dt_2*noise_ay;

    Q_(0, 1) = Q_(0, 3) = 0;
    Q_(1, 0) = Q_(1, 2) = 0;
    Q_(2, 1) = Q_(2, 3) = 0;
    Q_(3, 0) = Q_(3, 2) = 0;
}


Eigen::VectorXd FusionEKF::getZPredictRadar(Eigen::VectorXd x)
{
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    float rho, phi, rho_dot;
    rho = sqrt(px * px + py * py);
    if(fabs(rho) < 0.0001){
        phi = 0;
        rho_dot = 0;
    }
    else{
        phi = atan2(py, px);
        rho_dot = (px*vx + py*vy) / sqrt(px*px+py*py);
    }

    VectorXd result = VectorXd(3);
    result << rho, phi, rho_dot;

    return result;
}