#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <math.h>

struct Quaternions {
    float r;
    float i;
    float j;
    float k;
};

struct Coordinates {
    float x;
    float y;
    float z;
};

struct DataPoint {
    struct Quaternions quat;
    struct Coordinates linaccel;
    struct Coordinates grav;
    double time;
};

struct DataOut {                // 36 bytes total
    struct Coordinates pos;        // 12 bytes
    struct Quaternions quat;    // 16 bytes
    double time;                // 8 bytes
};


#endif