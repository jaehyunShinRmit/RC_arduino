#ifndef FARMBOTSENSOR_H
#define FARMBOTSENSOR_H


class FarmbotSensor
{
public:
    FarmbotSensor();
    Oriantation();
    //Initial condition
    long initRoll;
    long initPitch;
    long initHeading;
    long initLat;
    long initLon;

    //Heading,Pitch,Roll in Degree
    long roll_deg;
    long pitch_deg;
    long heading_deg;

    //Heading,Pitch,Roll in Radian
    long roll;
    long pitch;
    long heading;

    // Acc Gyro Mag
    long ax;
    long ay;
    long az;
    long gx;
    long gy;
    long gz;
    long mx;
    long my;
    long mz;

    // GPS
    long latitude;
    long longitude;
    long speed;
    
};

#endif // FARMBOTSENSOR_H
