#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>
#include "data_structures.h"

// Quaternion Methods (Absolute Orientation)
void ConvertQuaternionToRotationMatrix(struct Quaternions quaternion, BLA::Matrix<4, 4>& rotationMatrix);
void ConvertLocalToGlobalCoords(struct Coordinates uncorrectedAccel, struct Coordinates correctedAceel, BLA::Matrix<4, 4>& rotationMatrix);

// Position Methods
void CorrectAccel(struct Coordinates accel);
void UpdatePosition(struct Coordinates correctedAccel, uint32_t timeSinceLastUpdate_us);

// Print Methods
void PrintGravityVector(struct Coordinates gravity);
void PrintCurrentPosition();
void GetAndPrintAllReadings();
void PrintDetailedDeadReckoning(struct Coordinates accel);



#endif