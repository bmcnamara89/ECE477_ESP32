#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>
#include "data_structures.h"

// Quaternion Methods (Absolute Orientation)
void ConvertQuaternionToRotationMatrix(struct Quaternions quaternion);
struct Coordinates ConvertLocalToGlobalCoords(struct Coordinates uncorrectedAccel);

// Position Methods
void CorrectAccel(struct Coordinates accel, struct Coordinates grav);
float UpdatePosition(struct Coordinates correctedAccel);

// Print Methods
void PrintGravityVector(struct Coordinates gravity);
void PrintCurrentPosition();
void GetAndPrintAllReadings();
void PrintDetailedDeadReckoning(struct Coordinates pos, struct Coordinates vel, struct Coordinates accel);

//Matrix Methods
int InvertMatrix(float invOut[4][4]);
void MatMul(float in4by1[4], float out4by1[4]);

#endif