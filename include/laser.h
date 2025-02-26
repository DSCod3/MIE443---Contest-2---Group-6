#ifndef LASER_H
#define LASER_H

#include <sensor_msgs/LaserScan.h>

// Structure to hold key distance measurements.
struct DistancesStruct {
    float frontRay;
    float leftRay;
    float rightRay;
};

extern DistancesStruct distances;

// Callback function to update distance values.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif
