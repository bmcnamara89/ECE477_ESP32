#include "sensor.h"
#include "data_structures.h"


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
void UpdatePosition(struct Coordinates& correctedAccel, uint32_t timeSinceLastUpdate_us)
{
    double accelTermX, accelTermY, accelTermZ;

    const double errorMargin = 0.02;
    const double neg_errorMargin = -0.02;

    const double errorMarginVel = 0.0001;
    const double neg_errorMarginVel = -0.0001;

    if (correctedAccel.x < errorMargin && correctedAccel.x > neg_errorMargin)
    {
        correctedAccel.x = 0.0f;
        numZeroX++;
    }
    if (correctedAccel.y < errorMargin && correctedAccel.y > neg_errorMargin)
    {
        correctedAccel.y = 0.0f;
        numZeroY++;
    }
    if (correctedAccel.z < errorMargin && correctedAccel.z > neg_errorMargin)
    {
        correctedAccel.z = 0.0f;
        numZeroZ++;
    }


    if (velX < errorMarginVel && velX > neg_errorMarginVel) 
    {
        velX = 0.0f;
    }
    if (velY < errorMarginVel && velY > neg_errorMarginVel) 
    {
        velY = 0.0f;
    }
    if (velZ < errorMarginVel && velZ > neg_errorMarginVel) 
    {
        velZ = 0.0f;
    }
    

    if (numZeroX >= 3)
    {
        numZeroX = 0;
        velX = 0.0f;
        // posX = 0.0f;
    }
    if (numZeroY >= 3)
    {
        numZeroY = 0;
        velY = 0.0f;
        // posY = 0.0f;
    }
    if (numZeroZ >= 3)
    {
        numZeroZ = 0;
        velZ = 0.0f;
        // posZ = 0.0f;
    }

    samplesTakenSinceReset++;


    if (GetNetAcceleration(correctedAccel) > 1 && samplesTakenSinceReset > 100)
    {
        posX = 0;
        posY = 0;
        posZ = 0;
        velX = 0.0f;
        velY = 0.0f;
        velZ = 0.0f;
        printf("777.77\n");

        samplesTakenSinceReset = 0;
    }

    double timeSinceLastUpdate_sec = timeSinceLastUpdate_us * 0.000001;
    double timeSinceLastUpdate_sec_squared = pow(timeSinceLastUpdate_sec, 2);
    double accelTermCoeff = timeSinceLastUpdate_sec_squared * 0.5;



    velX += timeSinceLastUpdate_sec * correctedAccel.x;
    velY += timeSinceLastUpdate_sec * correctedAccel.y;
    velZ += timeSinceLastUpdate_sec * correctedAccel.z;

    accelTermX = correctedAccel.x * accelTermCoeff;
    accelTermY = correctedAccel.y * accelTermCoeff;
    accelTermZ = correctedAccel.z * accelTermCoeff;

    posX += (velX * timeSinceLastUpdate_sec + accelTermX);
    posY += (velY * timeSinceLastUpdate_sec + accelTermY);
    posZ += (velZ * timeSinceLastUpdate_sec + accelTermZ);
}


/**
 * @brief Gets the Euler Rotation instead of Quaternion. This is a different way of seeing 3D orientation
 * 
 * @param eulerRotations this is populated with the newly updated values
 * @param accel the acceleration readings to update the filter with
 * @param gyro the gyroscope readings to update the filter with
 * @param magnet the compass readings to update the filter with
 * 
 * @returns bool - if the eulerRotations struct was successfully (true) populated or not (false)
 */
bool GetEulerRotation(EulerRotations& eulerRotations, struct Coordinates& accel, struct Coordinates& gyro, struct Coordinates& magnet)
{
    Quaternion quaternion;
    if (GetQuaternion(quaternion))
    {
        float t0 = 2.0 * (quaternion.real * quaternion.i + quaternion.j * quaternion.k);
        float t1 = 1.0 - 2.0 * (quaternion.i * quaternion.i + quaternion.j * quaternion.j);
        eulerRotations.roll = UnitConversions::RadiansToDegrees(atan2(t0, t1));

        float t2 = +2.0 * (quaternion.real * quaternion.j - quaternion.k * quaternion.i);
        
        t2 = t2 > 1 ? 1.0 : t2;
        t2 = t2 < -1 ? -1.0 : t2;
        
        eulerRotations.pitch = UnitConversions::RadiansToDegrees(asin(t2));

        float t3 = +2.0 * (quaternion.real * quaternion.k + quaternion.i * quaternion.j);
        float t4 = +1.0 - 2.0 * (quaternion.j * quaternion.j + quaternion.k * quaternion.k);
        eulerRotations.yaw = UnitConversions::RadiansToDegrees(atan2(t3, t4));

        return true;
    }

    return false;
}


/**
 * @brief Gets the Euler Rotation instead of Quaternion. This is a different way of seeing 3D orientation
 * 
 * @param eulerRotations this is populated with the newly updated values
 * @param quaternion the quaternion to use to convert
 * 
 * @returns bool - if the eulerRotations struct was successfully (true) populated or not (false)
 */
bool GetEulerRotation(EulerRotations& eulerRotations, Quaternion& quaternion)
{
    float t0 = 2.0 * (quaternion.real * quaternion.i + quaternion.j * quaternion.k);
    float t1 = 1.0 - 2.0 * (quaternion.i * quaternion.i + quaternion.j * quaternion.j);
    eulerRotations.roll = UnitConversions::RadiansToDegrees(atan2(t0, t1));

    float t2 = 2.0 * (quaternion.real * quaternion.j - quaternion.k * quaternion.i);
    
    t2 = t2 > 1 ? 1.0 : t2;
    t2 = t2 < -1 ? -1.0 : t2;
    
    eulerRotations.pitch = UnitConversions::RadiansToDegrees(asin(t2));

    float t3 = 2.0 * (quaternion.real * quaternion.k + quaternion.i * quaternion.j);
    float t4 = 1.0 - 2.0 * (quaternion.j * quaternion.j + quaternion.k * quaternion.k);
    eulerRotations.yaw = UnitConversions::RadiansToDegrees(atan2(t3, t4));

    return true;
}


void ConvertLocalToGlobalCoords(struct Coordinates& uncorrectedAccel, struct Coordinates& correctedAccel, BLA::Matrix<4, 4>& rotationMatrix)
{
    bool didWork = Invert(rotationMatrix);

    BLA::Matrix<4, 1> accelerationMatrix = {uncorrectedAccel.x, uncorrectedAccel.y, uncorrectedAccel.z, 0};

    BLA::Matrix<4, 1> accelReferenced = rotationMatrix * accelerationMatrix;

    correctedAccel.x = accelReferenced(0);
    correctedAccel.y = accelReferenced(1);
    correctedAccel.z = accelReferenced(2);

    if (!didWork)
    {
        printf("OH NOES!!\n");
    }
}


void ConvertLocalToGlobalCoords(struct Coordinates& uncorrectedAccel, struct Coordinates& correctedAccel, EulerRotations& euler)
{
    correctedAccel.x = (float) (uncorrectedAccel.x*(cos(euler.roll)*cos(euler.yaw)+sin(euler.roll)*sin(euler.pitch)*sin(euler.yaw)) + uncorrectedAccel.y*(cos(euler.pitch)*sin(euler.yaw)) + uncorrectedAccel.z*(-sin(euler.roll)*cos(euler.yaw)+cos(euler.roll)*sin(euler.pitch)*sin(euler.yaw)));
    correctedAccel.y = (float) (uncorrectedAccel.x*(-cos(euler.roll)*sin(euler.yaw)+sin(euler.roll)*sin(euler.pitch)*cos(euler.yaw)) + uncorrectedAccel.y*(cos(euler.pitch)*cos(euler.yaw)) + uncorrectedAccel.z*(sin(euler.roll)*sin(euler.yaw)+ cos(euler.roll)*sin(euler.pitch)*cos(euler.yaw)));
    correctedAccel.z = (float) (uncorrectedAccel.x*(sin(euler.roll)*cos(euler.pitch)) + uncorrectedAccel.y*(-sin(euler.pitch)) + uncorrectedAccel.z*(cos(euler.roll)*cos(euler.pitch)));
}

void ConvertQuaternionToEulerAngles(Quaternion& quaternion, EulerRotations& euler)
{
    float w = quaternion.real;
    float x = quaternion.i;
    float y = quaternion.j;
    float z = quaternion.k;



    euler.yaw   = UnitConversions::RadiansToDegrees(atan2(2.0f * (x * y + w * z), w * w + x * x - y * y - z * z));
    euler.pitch = UnitConversions::RadiansToDegrees(-asin(2.0f * (x * z - w * y)));
    euler.roll = UnitConversions::RadiansToDegrees(atan2(2.0f * (w * x + y * z), w * w - x * x - y * y + z * z));
}

void ConvertQuaternionToRotationMatrix(Quaternion& quaternion, BLA::Matrix<4, 4>& rotationMatrix)
{
    float x_squared = pow(quaternion.i, 2);
    float y_squared = pow(quaternion.j, 2);
    float z_squared = pow(quaternion.k, 2);

    float xy2 = 2 * quaternion.i * quaternion.j;
    float xz2 = 2 * quaternion.i * quaternion.k;
    float yz2 = 2 * quaternion.j * quaternion.k;

    float sx2 = 2 * quaternion.i * quaternion.real;
    float sy2 = 2 * quaternion.j * quaternion.real;
    float sz2 = 2 * quaternion.k * quaternion.real;

    rotationMatrix(0, 0) = 1 - (2 * y_squared) - (2 * z_squared);
    rotationMatrix(0, 1) = -1 * (xy2 - sz2);
    rotationMatrix(0, 2) = xz2 + sy2;
    rotationMatrix(0, 3) = 0;

    

    rotationMatrix(1, 0) = -1 * (xy2 + sz2);
    rotationMatrix(1, 1) = 1 - (2 * x_squared) - (2 * z_squared);
    rotationMatrix(1, 2) = yz2 - sx2;
    rotationMatrix(1, 3) = 0;


    rotationMatrix(2, 0) = xz2 - sy2;
    rotationMatrix(2, 1) = yz2 + sx2;
    rotationMatrix(2, 2) = 1 - (2 * x_squared) - (2 * y_squared);
    rotationMatrix(2, 3) = 0;


    rotationMatrix(3, 0) = 0;
    rotationMatrix(3, 1) = 0;
    rotationMatrix(3, 2) = 0;
    rotationMatrix(3, 3) = 1;




    // rotationMatrix.row1[0] = 1 - (2 * y_squared) - (2 * z_squared);
    // rotationMatrix.row1[1] = xy2 - sz2;
    // rotationMatrix.row1[2] = xz2 + sy2;
    // rotationMatrix.row1[3] = 0;


    // rotationMatrix.row2[0] = xy2 + sz2;
    // rotationMatrix.row2[1] = 1 - (2 * x_squared) - (2 * z_squared);
    // rotationMatrix.row2[2] = yz2 - sx2;
    // rotationMatrix.row2[3] = 0;


    // rotationMatrix.row3[0] = xz2 - sy2;
    // rotationMatrix.row3[1] = yz2 - sx2;
    // rotationMatrix.row3[2] = 1 - (2 * x_squared) - (2 * y_squared);
    // rotationMatrix.row3[3] = 0;


    // rotationMatrix.row4[0] = 0;
    // rotationMatrix.row4[1] = 0;
    // rotationMatrix.row4[2] = 0;
    // rotationMatrix.row4[3] = 1;
}


void PrintEulerRotations(struct EulerRotations eulerRotations)
{
    printf("%f", eulerRotations.roll);
    printf("/");
    printf("%f", eulerRotations.pitch);
    printf("/");
    printf("%f"), (eulerRotations.yaw);
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



void SensorFusion::PrintGravityVector(struct Coordinates gravity)
{
    printf("Gravity Vector in m/s/s (x, y, z):  ");
    printf("%f", gravity.x);
    printf(", ");
    printf("%f", gravity.y);
    printf(", ");
    printf("%f", gravity.z);
}