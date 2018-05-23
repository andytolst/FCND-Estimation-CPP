# Estimation Project #

This is the implementation of FCND Estimation project.

### Step 1: Sensor Noise ###

1. Capture `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data)
2. Calculate standard deviation using Google Sheets:
![stdev](images/stdev.png)

3. Put results to `config/6_Sensornoise.txt`

4. Run simulator ans check the result:
![sensor noise](images/sensor_noise.png)


### Step 2: Attitude Estimation ###

1. In `QuadEstimatorEKF.cpp`, update function `UpdateFromIMU()` as described in **7.1.2 Nonlinear Complementary Filter**:

![quaternion](images/quaternion.png)

- create a quaternion `qt` from Euler angles using `FromEuler123_RPY` function:
   `Quaternion<float> qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6))`
- use `IntegrateBodyRate()` function (which is just multiplication of 2 quaternions) to integrate gyro rate
- use `ToEulerRPY()` to convert resulting quaternion to Euler angles Roll, Pitch and Yaw
- normalize yaw to -pi .. pi
- the rest of the code (already implemented) completes the filter:
![filter](images/roll_pitch_filter.png)
 

2. Run simulator ann check:
![attitude](images/attitude.png)

### Step 3: Prediction Step ###

1. Implement the state prediction step in the `PredictState()` functon as described in **7.2 Transition Model**:
![predict](images/predict.png)
taking into account that control vector `ut` is x,y,z accelerations in body frame + Yaw rate:
![ut](images/conrol_vector.png)
Note, we don't really have to calculate the rotation matrix `Rbg`, and rotate the first part of control vector `ut` using `Rotate_BtoI()` function of the attitude quaternion.
```
  // X,Y,Z
  predictedState[0] = predictedState[0] + predictedState[3] * dt;
  predictedState[1] = predictedState[1] + predictedState[4] * dt;
  predictedState[2] = predictedState[2] + predictedState[5] * dt;

  // velocities X, Y, Z
  V3F accelXYZ = attitude.Rotate_BtoI(accel);
  predictedState[3] = predictedState[3] + accelXYZ.x * dt;
  predictedState[4] = predictedState[4] + accelXYZ.y * dt;
  predictedState[5] = predictedState[5] - CONST_GRAVITY * dt + accelXYZ.z * dt;
```

2. Implement function `GetRbgPrime()`, just fill coefficients as following:
![rbgprime](images/rbgprime.png)
3. Implement `Predict()` function:
- fill `gPrime` matrix as following:
![gprime](images/gprime.png)
```
  gPrime(0,3) = dt;
  gPrime(1,4) = dt;
  gPrime(2,5) = dt;

  VectorXf ut(3);
  ut << accel[0], accel[1], accel[2];
  gPrime.block<3,1>(3,6) = RbgPrime * ut * dt;
```
- update covarinace using matrix multiplication:
![covariance](images/covariance.png)
```
  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```
4. Tune the process parameters in `QuadEstimatorEKF.txt` 

![covariance2](images/predict_covariance.png)


### Step 4: Magnetometer Update ###

1. Implement magnetometer update function `UpdateFromMag()` as described in **7.3.2 Magnetometer**:
![mag](images/mag.png)
```
  hPrime(0,6) = 1;
  zFromX(0) = ekfState(6);
```
then normalize the difference between your measured and estimated yaw
2. Tune `QYawStd` parameter:
![mag_update](images/mag_result.png)


### Step 5: Closed Loop + GPS Update ###

1. Implement the EKF GPS Update in the function `UpdateFromGPS()` as described in **7.3.1 GPS**:
![gps](images/gps.png)
```
  hPrime.block<6,6>(0,0).setIdentity();
  zFromX = ekfState.head(6);
```
2. Run with default controller:



### Step 6: Adding Your Controller ###

1. Replace `QuadController.cpp` and `QuadControlParams.txt` with the controller / params from the last project.

2. Tune the gains in `QuadControlParams.txt`


