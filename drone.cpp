#include "drone.h"

const char cmd_vel_topic[] = "/cmd_vel";
const char odometry_topic[] = "/ardrone/odometry"; // ARDrone
//const char odometry_topic[] = "/ground_truth/state"; //Gazebo


Drone::Drone() {

        std::cout << "init drone" << std::endl;
        upDateOdometry();
        upDateOdometry();
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}


Drone::~Drone() {

    if(ros::isStarted()) {
        std::cout<< "" << std::endl; // for soul xD
        ROS_INFO_STREAM("The application was closed --> closing ROS");
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();

}


void Drone::forward(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("forward: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000,true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}


void Drone::backward(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("backward: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.linear.x = -1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}

void Drone::left(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("left: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.linear.y = 1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}


void Drone::right(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("right: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;

    msg.linear.y = -1*persentage_of_cmd_vel/100;

    vel_pub.publish(msg);

}


int Drone::sign(double value) {

    return value/std::abs(value);

}


void Drone::angularRotate(double angular) {

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;

    double currentAngular = getEulerAngles().zAxis;
    double oldAngular = getEulerAngles().zAxis;
    double error = 0;
    double pwm = 0;
    double Kp = 0.02;

    double error_x = 0 - getEulerAngles().xAxis;
    double error_y = 0 - getEulerAngles().yAxis;

    while(abs(angular-error) > 5) {

        error_x = 0 - getEulerAngles().xAxis;
        error_y = 0 - getEulerAngles().yAxis;


        currentAngular = getEulerAngles().zAxis;
        error = currentAngular - oldAngular;
        std::cout << pwm << std::endl;

        pwm = (angular - error)*Kp;

        if(std::abs(pwm) > 1) pwm = sign(pwm)*1;
        msg.angular.z = pwm;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        vel_pub.publish(msg);

        if (std::abs(error_x) > 0.3) {

            if(std::abs(error_x) > 0.15) error_x = sign(error_x)*0.15;
            msg.angular.z = 0;
            msg.angular.x = error_x;
            msg.angular.y = 0;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            //std::cout << error_x << std::endl;

        }

        if (std::abs(error_y) > 0.3) {

            if(std::abs(error_y) > 0.15) error_y = sign(error_y)*0.15;
            msg.angular.y = error_y;
            msg.angular.z = 0;
            msg.angular.x = 0;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            //std::cout << "y: "<< error_y << std::endl;
        }

    }

    msg.angular.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    vel_pub.publish(msg);

}


void Drone::setAngular(char axis, double angular) {

    if (axis == 'z') {

        vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
        while(vel_pub.getNumSubscribers() < 1) {
            sleep(0.01);
        }
        geometry_msgs::Twist msg;

        double currentAngular = getEulerAngles().zAxis;
        std::cout << "Current angle(start): " << currentAngular << std::endl;
        double error = angular - currentAngular;
        double pwm = 0;
        double Kp = 0.5;

        double error_x = 0 - getEulerAngles().xAxis;
        double error_y = 0 - getEulerAngles().yAxis;

        while(abs(error) > 5) {

            error_x = 0 - getEulerAngles().xAxis;
            error_y = 0 - getEulerAngles().yAxis;


            currentAngular = getEulerAngles().zAxis;
            error = angular - currentAngular;

            if (std::abs(error) > 180) error = error - sign(error) * 360;

            pwm = error*Kp;

            if(std::abs(pwm) > 1) pwm = sign(pwm)*1;
            //std::cout << "z error:" << error << "   Current angle: " << currentAngular << "   pwm: " << pwm << std::endl;
            msg.angular.z = pwm;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0;
            vel_pub.publish(msg);

            if (std::abs(error_x) > 0.3) {

                if(std::abs(error_x) > 0.01) error_x = sign(error_x)*0.01;
                msg.angular.z = 0;
                msg.angular.x = error_x;
                msg.angular.y = 0;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                //std::cout << error_x << std::endl;

            }

            if (std::abs(error_y) > 0.3) {

                if(std::abs(error_y) > 0.01) error_y = sign(error_y)*0.01;
                msg.angular.y = error_y;
                msg.angular.z = 0;
                msg.angular.x = 0;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                //std::cout << "y: "<< error_y << std::endl;
            }

            std::cout << error_x << "  " << error_y << std::endl;
        }

        msg.angular.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;

        vel_pub.publish(msg);

    }

}


void Drone::heightSpeed(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("hight speed: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;

    msg.linear.z = persentage_of_cmd_vel/100;

    vel_pub.publish(msg);

}


void Drone::angular(char axis, double persentage_of_cmd_vel) {

    if (axis == 'x') {

        vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
        while(vel_pub.getNumSubscribers() < 1) {
            sleep(0.01);
        }
        geometry_msgs::Twist msg;

        msg.angular.x = persentage_of_cmd_vel/100;
        vel_pub.publish(msg);

    }

    if (axis == 'y') {

        vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
        while(vel_pub.getNumSubscribers() < 1) {
            sleep(0.01);
        }
        geometry_msgs::Twist msg;

        msg.angular.y = persentage_of_cmd_vel/100;
        vel_pub.publish(msg);

    }

    if (axis == 'z') {

        ROS_INFO_STREAM("z angular: " << persentage_of_cmd_vel << "%");

        vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
        while(vel_pub.getNumSubscribers() < 1) {
            sleep(0.01);
        }
        geometry_msgs::Twist msg;

        msg.angular.z = persentage_of_cmd_vel/100;
        vel_pub.publish(msg);

    }

}


void Drone::reset() {

    ROS_INFO_STREAM("reset");

    vel_pub = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty msg;
    vel_pub.publish(msg);

}


void Drone::take_off() {

    ROS_INFO_STREAM("take_off");

    vel_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty msg;
    vel_pub.publish(msg);

}


void Drone::land() {

    ROS_INFO_STREAM("land");

    vel_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty msg;
    vel_pub.publish(msg);

}


void Drone::hover() {

    ROS_INFO_STREAM("hover");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    geometry_msgs::Twist velocity;

    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    vel_pub.publish(velocity);

}


Drone::EulerAngle Drone::toEulerAngle() {

    // roll (x-axis rotation)
    double sinr = +2.0 * (w * x + y * z);
    double cosr = +1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(sinr, cosr);
    roll = roll*180/M_PI;

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    pitch = pitch*180/M_PI;

    // yaw (z-axis rotation)
    double siny = +2.0 * (w * z + x * y);
    double cosy = +1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny, cosy);
    yaw = yaw*180/M_PI;
    if (yaw < 0) {
        yaw = 360+yaw;
    }


    Drone::EulerAngle axis;
    axis.xAxis = roll;
    axis.yAxis = pitch;
    axis.zAxis = yaw;

    return axis;

}


Drone::EulerAngle Drone::getEulerAngles() {

    upDateOdometry();

    Drone::EulerAngle axis;
    axis = toEulerAngle();

    return axis;

}


Drone::Position Drone::getPosition() {

    upDateOdometry();

    Drone::Position position;
    position.x_coord = XCurrentCoord;
    position.y_coord = YCurrentCoord;
    position.z_coord = ZCurrentCoord;

    return position;

}


void Drone::upDateOdometry() {

    gr_sub = nh.subscribe(odometry_topic, 1000, &Drone::goal_callback, this);
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

}

void Drone::goal_callback(const nav_msgs::Odometry &msg){

    x = msg.pose.pose.orientation.x;
    y = msg.pose.pose.orientation.y;
    z = msg.pose.pose.orientation.z;
    w = msg.pose.pose.orientation.w;

    XCurrentCoord = msg.pose.pose.position.x;
    YCurrentCoord = msg.pose.pose.position.y;
    ZCurrentCoord = msg.pose.pose.position.z;

}

void Drone::setLinearVelocity(double xpwm, double ypwm, double zpwm) {

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    geometry_msgs::Twist velocity;

    velocity.linear.x = xpwm;
    velocity.linear.y = ypwm;
    velocity.linear.z = zpwm;

    vel_pub.publish(velocity);

}


void Drone::setAngularVelocity(double z) {

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    geometry_msgs::Twist velocity;

    velocity.angular.z = z;

    vel_pub.publish(velocity);

}
