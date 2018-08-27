# Chuan's Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project I will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Own project work based on original project repository
### FusionEKF.cpp
The main code changes I have for FusionEKF.cpp are:
* Initialize state vectors with first measuremens from Lidar or Radar sensor:
```c++
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0] * cos (measurement_pack.raw_measurements_[1]);
      ekf_.x_(1) = measurement_pack.raw_measurements_[0] * sin (measurement_pack.raw_measurements_[1]);
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
```
* Calculate state transition matrix F and process noise covariance matrix Q for prediction step based on dt:
```C++
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;  

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			  0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			  dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			  0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
```
* Update measure matrix H and measurement covariance matrix R for kalman filter updating step, for Radar sensor measurements H should be linearized jacobian matrix Hj, and measurement covariance matrix R is differentiated for Radar and Lidar updating steps. For Lidar measurement update, it is calling Update function in kalman_filter.cpp, for radar measurement update, it is calling UpdateEKF function:
```c++
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    
    // Update measurement (Jacobian) matrix for Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    
    // Update measurement covariance matrix - radar
    ekf_.R_ = R_radar_;
    
    // Call Radar update function
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // Laser updates
    
    // Update meaurement matrix for Radar updates
    ekf_.H_ = H_laser_;
    
    // Update measurement covariance matrix - lidar
    ekf_.R_ = R_laser_;

    // Call Lidar update function
    ekf_.Update(measurement_pack.raw_measurements_);
  }
```
### kalman_filter.cpp
The main code changes I have for kalman_filter.cpp are:
* Update prediction function with codes proivded from class:
```c++
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
```
* Kalman filter update function for lidar measurements:
```c++
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  #define PI 3.14159265;
  
  VectorXd z_pred;
  z_pred = VectorXd(3);
  z_pred(0) = sqrt (x_(0)*x_(0) + x_(1)*x_(1));
  z_pred(1) = atan2 (x_(1), x_(0));
  z_pred(2) = (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt (x_(0)*x_(0) + x_(1)*x_(1));
  VectorXd y = z - z_pred;
  
  //Adjust phi in y to stay inside -PI and PI
  while ((y(1) > 3.14159265) || (y(1) < -3.14159265))
  {
    if(y(1) > 3.14159265)
    {
      y(1) = y(1) - 2 * PI;
    }
    else
    {
      y(1) = y(1) + 2 * PI;
    }
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  
  
}
```
In above extended kalman filter code, z_pred is calculated using z'=h(x'), and when calculating phi in z', angle is normalized to -pi and pi.
### tools.cpp
The main code changes I have for tools.cpp are:
* Update the code for calculating Jacobian matrix for Extened Kalman Filter updating function based on codes from class exercise:
```c++
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;          
	return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
		-(py/c1), (px/c1), 0, 0,
		py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;  
}
```
* Update the code for calculating RMSE based on codes from class exercise:
```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
	      || estimations.size() == 0){
	cout << "Invalid estimation or ground_truth data" << endl;
	return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

	VectorXd residual = estimations[i] - ground_truth[i];

	//coefficient-wise multiplication
	residual = residual.array()*residual.array();
	rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}
```
