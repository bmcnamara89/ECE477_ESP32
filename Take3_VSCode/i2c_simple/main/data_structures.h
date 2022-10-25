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

struct DataOut {
    struct Position pos;
    struct Quaternions quat;
    double time;
}