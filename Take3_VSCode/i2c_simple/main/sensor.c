#include <stdio.h>
#include <math.h>
#include "sensor.h"
#include "data_structures.h"

//Global Variables
extern struct Coordinates globalPosition;
extern struct Coordinates globalVelocity;
extern double globalLastTime;
extern double globalTimeSinceLastPoint;
extern float rotationMatrix [4][4];

/**
 * @brief Corrects the raw acceleration readings and repopulates the passed in struct with the corrected values
 * 
 * @param accel the acceleration readings to correct. When the function call is finished this will now be populated with the corrected values
 */
void CorrectAccel(struct Coordinates accel, struct Coordinates grav)
{
    accel.x -= grav.x;
    accel.y -= grav.y;
    accel.z -= grav.z;
}


/**
 * @brief Updates the current position estimate using Dead Reckoning
 * 
 * @param correctedAccel the acceleration after it has been corrected from gravity
 * @param timeSinceLastUpdate_us the time in microseconds since this method was last called.
 */
void UpdatePosition(struct Coordinates correctedAccel)
{
    double accelTermX, accelTermY, accelTermZ;

    const double errorMargin = 0.02;
    const double neg_errorMargin = -0.02;

    const double errorMarginVel = 0.0001;
    const double neg_errorMarginVel = -0.0001;

    if (correctedAccel.x < errorMargin && correctedAccel.x > neg_errorMargin)
    {
        correctedAccel.x = 0.0f;
    }
    if (correctedAccel.y < errorMargin && correctedAccel.y > neg_errorMargin)
    {
        correctedAccel.y = 0.0f;
    }
    if (correctedAccel.z < errorMargin && correctedAccel.z > neg_errorMargin)
    {
        correctedAccel.z = 0.0f;
    }


    if (globalVelocity.x < errorMarginVel && globalVelocity.x > neg_errorMarginVel) 
    {
        globalVelocity.x = 0.0f;
    }
    if (globalVelocity.y < errorMarginVel && globalVelocity.y > neg_errorMarginVel) 
    {
        globalVelocity.y = 0.0f;
    }
    if (globalVelocity.z < errorMarginVel && globalVelocity.z > neg_errorMarginVel) 
    {
        globalVelocity.z = 0.0f;
    }
    
    double accelTermCoeff = globalTimeSinceLastPoint * globalTimeSinceLastPoint * 0.5;

    globalVelocity.x += globalTimeSinceLastPoint * correctedAccel.x;
    globalVelocity.y += globalTimeSinceLastPoint * correctedAccel.y;
    globalVelocity.z += globalTimeSinceLastPoint * correctedAccel.z;

    accelTermX = correctedAccel.x * accelTermCoeff;
    accelTermY = correctedAccel.y * accelTermCoeff;
    accelTermZ = correctedAccel.z * accelTermCoeff;

    globalPosition.x += (globalVelocity.x * globalTimeSinceLastPoint + accelTermX);
    globalPosition.y += (globalVelocity.y * globalTimeSinceLastPoint + accelTermY);
    globalPosition.z += (globalVelocity.z * globalTimeSinceLastPoint + accelTermZ);
}


void ConvertLocalToGlobalCoords(struct Coordinates uncorrectedAccel, struct Coordinates correctedAccel)
{
    float invOut[4][4];
    int didWork = InvertMatrix(invOut);

    float accelerationMatrix[4] = {uncorrectedAccel.x, uncorrectedAccel.y, uncorrectedAccel.z, 0};

    float accelReferenced[4] = rotationMatrix * accelerationMatrix;

    correctedAccel.x = accelReferenced[0];
    correctedAccel.y = accelReferenced[1];
    correctedAccel.z = accelReferenced[2];

    if (didWork == 0)
    {
        printf("OH NOES!!\n");
    }
}

void ConvertQuaternionToRotationMatrix(struct Quaternions quaternion)
{
    float x_squared = pow(quaternion.i, 2);
    float y_squared = pow(quaternion.j, 2);
    float z_squared = pow(quaternion.k, 2);

    float xy2 = 2 * quaternion.i * quaternion.j;
    float xz2 = 2 * quaternion.i * quaternion.k;
    float yz2 = 2 * quaternion.j * quaternion.k;

    float sx2 = 2 * quaternion.i * quaternion.r;
    float sy2 = 2 * quaternion.j * quaternion.r;
    float sz2 = 2 * quaternion.k * quaternion.r;

    rotationMatrix[0][0] = 1 - (2 * y_squared) - (2 * z_squared);
    rotationMatrix[0][1] = -1 * (xy2 - sz2);
    rotationMatrix[0][2] = xz2 + sy2;
    rotationMatrix[0][3] = 0;

    

    rotationMatrix[1][0] = -1 * (xy2 + sz2);
    rotationMatrix[1][1] = 1 - (2 * x_squared) - (2 * z_squared);
    rotationMatrix[1][2] = yz2 - sx2;
    rotationMatrix[1][3] = 0;


    rotationMatrix[2][0] = xz2 - sy2;
    rotationMatrix[2][1] = yz2 + sx2;
    rotationMatrix[2][2] = 1 - (2 * x_squared) - (2 * y_squared);
    rotationMatrix[2][3] = 0;


    rotationMatrix[3][0] = 0;
    rotationMatrix[3][1] = 0;
    rotationMatrix[3][2] = 0;
    rotationMatrix[3][3] = 1;
}

void PrintCurrentPosition(struct Coordinates pos)
{
    printf("%f", pos.x);
    printf("/");
    printf("%f", pos.y);
    printf("/");
    printf("%f", pos.z);
}

void PrintDetailedDeadReckoning(struct Coordinates pos, struct Coordinates vel, struct Coordinates accel)
{
    printf("Position(m) (x, y, z): (");
    printf("%f", pos.x);
    printf(", ");
    printf("%f", pos.y);
    printf(", ");
    printf("%f", pos.z);
    printf(")");

    printf(" Velocity(m/s): (");
    printf("%f", vel.x);
    printf(", ");
    printf("%f", vel.y);
    printf(", ");
    printf("%f", vel.z);

    printf(")  Accel(m/s/s) (x, y, z): ");
    printf("%f", accel.x);
    printf(", ");
    printf("%f", accel.y);
    printf(", ");
    printf("%f", accel.z);
}

void PrintGravityVector(struct Coordinates gravity)
{
    printf("Gravity Vector in m/s/s (x, y, z):  ");
    printf("%f", gravity.x);
    printf(", ");
    printf("%f", gravity.y);
    printf(", ");
    printf("%f", gravity.z);
}


//Adapted from: https://stackoverflow.com/a/1148405
int InvertMatrix(float invOut[4][4])
{
    float inv[4][4];
    float det;

    inv[0][0] = rotationMatrix[1][1]  * rotationMatrix[2][2] * rotationMatrix[3][3] - 
             rotationMatrix[1][1]  * rotationMatrix[2][3] * rotationMatrix[3][2] - 
             rotationMatrix[2][1]  * rotationMatrix[1][2]  * rotationMatrix[3][3] + 
             rotationMatrix[2][1]  * rotationMatrix[1][3]  * rotationMatrix[3][2] +
             rotationMatrix[3][1] * rotationMatrix[1][2]  * rotationMatrix[2][3] - 
             rotationMatrix[3][1] * rotationMatrix[1][3]  * rotationMatrix[2][2];

    inv[1][0] = -rotationMatrix[1][0]  * rotationMatrix[2][2] * rotationMatrix[3][3] + 
              rotationMatrix[1][0]  * rotationMatrix[2][3] * rotationMatrix[3][2] + 
              rotationMatrix[2][0]  * rotationMatrix[1][2]  * rotationMatrix[3][3] - 
              rotationMatrix[2][0]  * rotationMatrix[1][3]  * rotationMatrix[3][2] - 
              rotationMatrix[3][0] * rotationMatrix[1][2]  * rotationMatrix[2][3] + 
              rotationMatrix[3][0] * rotationMatrix[1][3]  * rotationMatrix[2][2];

    inv[2][0] = rotationMatrix[1][0]  * rotationMatrix[2][1] * rotationMatrix[3][3] - 
             rotationMatrix[1][0]  * rotationMatrix[2][3] * rotationMatrix[3][1] - 
             rotationMatrix[2][0]  * rotationMatrix[1][1] * rotationMatrix[3][3] + 
             rotationMatrix[2][0]  * rotationMatrix[1][3] * rotationMatrix[3][1] + 
             rotationMatrix[3][0] * rotationMatrix[1][1] * rotationMatrix[2][3] - 
             rotationMatrix[3][0] * rotationMatrix[1][3] * rotationMatrix[2][1];

    inv[3][0] = -rotationMatrix[1][0]  * rotationMatrix[2][1] * rotationMatrix[3][2] + 
               rotationMatrix[1][0]  * rotationMatrix[2][2] * rotationMatrix[3][1] +
               rotationMatrix[2][0]  * rotationMatrix[1][1] * rotationMatrix[3][2] - 
               rotationMatrix[2][0]  * rotationMatrix[1][2] * rotationMatrix[3][1] - 
               rotationMatrix[3][0] * rotationMatrix[1][1] * rotationMatrix[2][2] + 
               rotationMatrix[3][0] * rotationMatrix[1][2] * rotationMatrix[2][1];

    inv[0][1] = -rotationMatrix[0][1]  * rotationMatrix[2][2] * rotationMatrix[3][3] + 
              rotationMatrix[0][1]  * rotationMatrix[2][3] * rotationMatrix[3][2] + 
              rotationMatrix[2][1]  * rotationMatrix[0][2] * rotationMatrix[3][3] - 
              rotationMatrix[2][1]  * rotationMatrix[0][3] * rotationMatrix[3][2] - 
              rotationMatrix[3][1] * rotationMatrix[0][2] * rotationMatrix[2][3] + 
              rotationMatrix[3][1] * rotationMatrix[0][3] * rotationMatrix[2][2];

    inv[1][1] = rotationMatrix[0][0]  * rotationMatrix[2][2] * rotationMatrix[3][3] - 
             rotationMatrix[0][0]  * rotationMatrix[2][3] * rotationMatrix[3][2] - 
             rotationMatrix[2][0]  * rotationMatrix[0][2] * rotationMatrix[3][3] + 
             rotationMatrix[2][0]  * rotationMatrix[0][3] * rotationMatrix[3][2] + 
             rotationMatrix[3][0] * rotationMatrix[0][2] * rotationMatrix[2][3] - 
             rotationMatrix[3][0] * rotationMatrix[0][3] * rotationMatrix[2][2];

    inv[2][1] = -rotationMatrix[0][0]  * rotationMatrix[2][1] * rotationMatrix[3][3] + 
              rotationMatrix[0][0]  * rotationMatrix[2][3] * rotationMatrix[3][1] + 
              rotationMatrix[2][0]  * rotationMatrix[0][1] * rotationMatrix[3][3] - 
              rotationMatrix[2][0]  * rotationMatrix[0][3] * rotationMatrix[3][1] - 
              rotationMatrix[3][0] * rotationMatrix[0][1] * rotationMatrix[2][3] + 
              rotationMatrix[3][0] * rotationMatrix[0][3] * rotationMatrix[2][1];

    inv[3][1] = rotationMatrix[0][0]  * rotationMatrix[2][1] * rotationMatrix[3][2] - 
              rotationMatrix[0][0]  * rotationMatrix[2][2] * rotationMatrix[3][1] - 
              rotationMatrix[2][0]  * rotationMatrix[0][1] * rotationMatrix[3][2] + 
              rotationMatrix[2][0]  * rotationMatrix[0][2] * rotationMatrix[3][1] + 
              rotationMatrix[3][0] * rotationMatrix[0][1] * rotationMatrix[2][2] - 
              rotationMatrix[3][0] * rotationMatrix[0][2] * rotationMatrix[2][1];

    inv[0][2] = rotationMatrix[0][1]  * rotationMatrix[1][2] * rotationMatrix[3][3] - 
             rotationMatrix[0][1]  * rotationMatrix[1][3] * rotationMatrix[3][2] - 
             rotationMatrix[1][1]  * rotationMatrix[0][2] * rotationMatrix[3][3] + 
             rotationMatrix[1][1]  * rotationMatrix[0][3] * rotationMatrix[3][2] + 
             rotationMatrix[3][1] * rotationMatrix[0][2] * rotationMatrix[1][3] - 
             rotationMatrix[3][1] * rotationMatrix[0][3] * rotationMatrix[1][2];

    inv[1][2] = -rotationMatrix[0][0]  * rotationMatrix[1][2] * rotationMatrix[3][3] + 
              rotationMatrix[0][0]  * rotationMatrix[1][3] * rotationMatrix[3][2] + 
              rotationMatrix[1][0]  * rotationMatrix[0][2] * rotationMatrix[3][3] - 
              rotationMatrix[1][0]  * rotationMatrix[0][3] * rotationMatrix[3][2] - 
              rotationMatrix[3][0] * rotationMatrix[0][2] * rotationMatrix[1][3] + 
              rotationMatrix[3][0] * rotationMatrix[0][3] * rotationMatrix[1][2];

    inv[2][2] = rotationMatrix[0][0]  * rotationMatrix[1][1] * rotationMatrix[3][3] - 
              rotationMatrix[0][0]  * rotationMatrix[1][3] * rotationMatrix[3][1] - 
              rotationMatrix[1][0]  * rotationMatrix[0][1] * rotationMatrix[3][3] + 
              rotationMatrix[1][0]  * rotationMatrix[0][3] * rotationMatrix[3][1] + 
              rotationMatrix[3][0] * rotationMatrix[0][1] * rotationMatrix[1][3] - 
              rotationMatrix[3][0] * rotationMatrix[0][3] * rotationMatrix[1][1];

    inv[3][2] = -rotationMatrix[0][0]  * rotationMatrix[1][1] * rotationMatrix[3][2] + 
               rotationMatrix[0][0]  * rotationMatrix[1][2] * rotationMatrix[3][1] + 
               rotationMatrix[1][0]  * rotationMatrix[0][1] * rotationMatrix[3][2] - 
               rotationMatrix[1][0]  * rotationMatrix[0][2] * rotationMatrix[3][1] - 
               rotationMatrix[3][0] * rotationMatrix[0][1] * rotationMatrix[1][2] + 
               rotationMatrix[3][0] * rotationMatrix[0][2] * rotationMatrix[1][1];

    inv[0][3] = -rotationMatrix[0][1] * rotationMatrix[1][2] * rotationMatrix[2][3] + 
              rotationMatrix[0][1] * rotationMatrix[1][3] * rotationMatrix[2][2] + 
              rotationMatrix[1][1] * rotationMatrix[0][2] * rotationMatrix[2][3] - 
              rotationMatrix[1][1] * rotationMatrix[0][3] * rotationMatrix[2][2] - 
              rotationMatrix[2][1] * rotationMatrix[0][2] * rotationMatrix[1][3] + 
              rotationMatrix[2][1] * rotationMatrix[0][3] * rotationMatrix[1][2];

    inv[1][3] = rotationMatrix[0][0] * rotationMatrix[1][2] * rotationMatrix[2][3] - 
             rotationMatrix[0][0] * rotationMatrix[1][3] * rotationMatrix[2][2] - 
             rotationMatrix[1][0] * rotationMatrix[0][2] * rotationMatrix[2][3] + 
             rotationMatrix[1][0] * rotationMatrix[0][3] * rotationMatrix[2][2] + 
             rotationMatrix[2][0] * rotationMatrix[0][2] * rotationMatrix[1][3] - 
             rotationMatrix[2][0] * rotationMatrix[0][3] * rotationMatrix[1][2];

    inv[2][3] = -rotationMatrix[0][0] * rotationMatrix[1][1] * rotationMatrix[2][3] + 
               rotationMatrix[0][0] * rotationMatrix[1][3] * rotationMatrix[2][1] + 
               rotationMatrix[1][0] * rotationMatrix[0][1] * rotationMatrix[2][3] - 
               rotationMatrix[1][0] * rotationMatrix[0][3] * rotationMatrix[2][1] - 
               rotationMatrix[2][0] * rotationMatrix[0][1] * rotationMatrix[1][3] + 
               rotationMatrix[2][0] * rotationMatrix[0][3] * rotationMatrix[1][1];

    inv[3][3] = rotationMatrix[0][0] * rotationMatrix[1][1] * rotationMatrix[2][2] - 
              rotationMatrix[0][0] * rotationMatrix[1][2] * rotationMatrix[2][1] - 
              rotationMatrix[1][0] * rotationMatrix[0][1] * rotationMatrix[2][2] + 
              rotationMatrix[1][0] * rotationMatrix[0][2] * rotationMatrix[2][1] + 
              rotationMatrix[2][0] * rotationMatrix[0][1] * rotationMatrix[1][2] - 
              rotationMatrix[2][0] * rotationMatrix[0][2] * rotationMatrix[1][1];

    det = rotationMatrix[0][0] * inv[0][0] + rotationMatrix[0][1] * inv[1][0] + rotationMatrix[0][2] * inv[2][0] + rotationMatrix[0][3] * inv[3][0];

    if (det == 0)
        return 0;

    det = 1.0 / det;

    int r, c;
    for (r = 0; r < 4; r++)
    {
        for(c = 0; c < 4; c++)
        {
            invOut[r][c] = inv[r][c] * det;
        }
    }

    return 1;
}