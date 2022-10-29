#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>

// Quaternion Methods (Absolute Orientation)
bool GetEulerRotation(EulerRotations& eulerRotations, Quaternion& quaternion);
bool GetEulerRotation(EulerRotations& eulerRotations, DirectionalValues& accel, DirectionalValues& gyro, DirectionalValues& magnet);
void ConvertQuaternionToRotationMatrix(Quaternion& quaternion, BLA::Matrix<4, 4>& rotationMatrix);
void ConvertQuaternionToEulerAngles(Quaternion& quaternion, EulerRotations& euler);
void ConvertLocalToGlobalCoords(DirectionalValues& uncorrectedAccel, DirectionalValues& correctedAceel, BLA::Matrix<4, 4>& rotationMatrix);
void ConvertLocalToGlobalCoords(DirectionalValues& uncorrectedAccel, DirectionalValues& correctedAccel, EulerRotations& euler);

// Position Methods
//  bool GetGravityVector(DirectionalValues& gravityVector, Quaternion& quaternion);
void CorrectAccel(DirectionalValues& accel);
void UpdatePosition(DirectionalValues& correctedAccel, uint32_t timeSinceLastUpdate_us);

// Print Methods
void PrintGravityVector(DirectionalValues& gravity);
void PrintEulerRotations(EulerRotations& eulerRotations);
void PrintCurrentPosition();
void GetAndPrintAllReadings();
void PrintDetailedDeadReckoning(DirectionalValues& accel);



#endif