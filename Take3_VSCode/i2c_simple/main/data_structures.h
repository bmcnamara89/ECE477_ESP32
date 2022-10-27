#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <math.h>

struct Quaternions {
    float r;
    float i;
    float j;
    float k;
};

struct LinearAcceleration {
    float x;
    float y;
    float z;
};

struct Gravity {
    float x;
    float y;
    float z;
};

struct Position {
    float x;
    float y;
    float z;
};

struct DataPoint {
    struct Quaternions quat;
    struct LinearAcceleration linaccel;
    struct Gravity grav;
    double time;
};

struct DataOut {                // 36 bytes total
    struct Position pos;        // 12 bytes
    struct Quaternions quat;    // 16 bytes
    double time;                // 8 bytes
};


#endif