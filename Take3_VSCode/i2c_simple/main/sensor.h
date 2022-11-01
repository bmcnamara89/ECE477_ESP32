#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#include <math.h>
#include "data_structures.h"

// Quaternion Methods (Absolute Orientation)
void ConvertQuaternionToRotationMatrix(struct Quaternions quaternion);
void ConvertLocalToGlobalCoords(struct Coordinates uncorrectedAccel, struct Coordinates correctedAceel);

// Position Methods
void CorrectAccel(struct Coordinates accel, struct Coordinates grav);
void UpdatePosition(struct Coordinates correctedAccel);

// Print Methods
void PrintGravityVector(struct Coordinates gravity);
void PrintCurrentPosition();
void GetAndPrintAllReadings();
void PrintDetailedDeadReckoning(struct Coordinates pos, struct Coordinates vel, struct Coordinates accel);

//Matrix Methods
int InvertMatrix(float invOut[4][4]);



#endif