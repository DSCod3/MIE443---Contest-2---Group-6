#include "laser.h"
#include <algorithm>
#include <limits>

DistancesStruct distances;

// This is a simple example that picks:
/// - the middle range for the front,
/// - the maximum angle for left,
/// - and the minimum angle for right.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int n = msg->ranges.size();
    if (n == 0) return;

    // Get middle index for front.
    int mid = n / 2;
    distances.frontRay = msg->ranges[mid];

    // Get left (assume last reading corresponds to left side).
    distances.leftRay = msg->ranges[n-1];

    // Get right (assume first reading corresponds to right side).
    distances.rightRay = msg->ranges[0];

    // Optionally, you can add more robust checks:
    // For example, ignore NaN values and use min/max over a small window.
    // Here’s a simple alternative for the front reading:
    float front_min = std::numeric_limits<float>::max();
    int window = 5; // average over a window of 5 readings
    int start = std::max(0, mid - window/2);
    int end = std::min(n - 1, mid + window/2);
    for (int i = start; i <= end; ++i) {
        if (!std::isnan(msg->ranges[i]) && msg->ranges[i] < front_min)
            front_min = msg->ranges[i];
    }
    distances.frontRay = front_min;
}
