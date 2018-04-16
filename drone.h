#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <iostream>

#include <math.h>
#include <cmath>
#include <stdlib.h>

#include <nav_msgs/Odometry.h>
#include <iomanip>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include "ardrone_autonomy/Navdata.h"

#include "navdata.h"
#include <ctime>

#ifndef DRONE_H_
#define DRONE_H_

class Drone {

public:

    struct Euler_angle {

        double xAxis, yAxis, yaw;

    };

    struct Position {

        double x_coord, y_coord, z_coord;

    };

    struct Navdata navdata;

public:

    Drone();
    ~Drone();

    void reset();
    void take_off();
    void land();
    void hover();
  
private:

    int sign(double value);

    Euler_angle to_euler_angle();

    ros::NodeHandle nh;
    ros::Subscriber gr_sub;
    ros::Publisher vel_pub;

    double x, y, z, w;
    double XCurrentCoord, YCurrentCoord, ZCurrentCoord;

};

#endif /* DRONE_H_ */
