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

#ifndef DRONE_H_
#define DRONE_H_

class Drone {

public:

    struct EulerAngle {

        double xAxis, yAxis, zAxis;

    };

    struct Position {

        double x_coord, y_coord, z_coord;

    };

public:

    Drone();
    ~Drone();


    void angularRotate(double angular);
    void forward(double persentage_of_cmd_vel);
    void backward(double persentage_of_cmd_vel);
    void left(double persentage_of_cmd_vel);
    void right(double persentage_of_cmd_vel);
    void reset();
    void take_off();
    void hover();
    void land();
    void angular(char axis, double persentage_of_cmd_vel);
    void heightSpeed(double persentage_of_cmd_vel);
    void setAngular(char axis, double angular);
    void setLinearVelocity(double xpwm, double ypwm, double zpwm);
    void setAngularVelocity(double z);

    Position getPosition();
    EulerAngle getEulerAngles();

private:

    int sign(double value);

    EulerAngle toEulerAngle();

    void upDateOdometry();
    void goal_callback(const nav_msgs::Odometry & msg);

    ros::NodeHandle nh;
    ros::Subscriber gr_sub;
    ros::Publisher vel_pub;

    double x, y, z, w;
    double XCurrentCoord, YCurrentCoord, ZCurrentCoord;

};

#endif /* DRONE_H_ */

