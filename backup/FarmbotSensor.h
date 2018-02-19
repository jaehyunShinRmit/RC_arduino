#ifndef FARMBOTSENSOR_H
#define FARMBOTSENSOR_H


class FarmbotSensor
{
public:
    FarmbotSensor();
    Oriantation();
    //Initial condition
    float initRoll;
    float initPitch;
    float initHeading;
    float initLat;
    float initLon;

    //Heading,Pitch,Roll in Degree
    float roll_deg;
    float pitch_deg;
    float heading_deg;

    //Heading,Pitch,Roll in Radian
    float roll;
    float pitch;
    float heading;

    // Acc Gyro Mag
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
};

#endif // FARMBOTSENSOR_H
