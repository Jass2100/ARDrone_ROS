#include "drone.h"

const char cmd_vel_topic[] = "/cmd_vel";
//const char odometry_topic[] = "/ardrone/odometry"; // ARDrone
const char odometry_topic[] = "/ground_truth/state"; //Gazebo


Drone::Drone() {

        std::cout << "initialization drone control class" << std::endl;
        update_odometry();
        update_odometry();
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        Navdata navdata;
		navdata = get_navdata();

		std::cout << "" << std::endl;
		std::cout << "ARDrone information" << std::endl;
		std::cout << "________________________" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "Battery: " << navdata.batteryPercent << "%" << std::endl;
		std::cout << "State: " << navdata.state << std::endl;
		std::cout << "________________________" << std::endl;
		std::cout << "" << std::endl;

}


Drone::~Drone() {

    if(ros::isStarted()) {
        std::cout<< "" << std::endl; // for soul xD
        ROS_INFO_STREAM("closing ROS...");
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();

}


void Drone::yaw_rotate(double persentage_of_cmd_vel) {

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.angular.z = 1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}


void Drone::linear_x(double persentage_of_cmd_vel) {

    //ROS_INFO_STREAM("linear_x: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000,true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}


void Drone::linear_y(double persentage_of_cmd_vel) {

    //ROS_INFO_STREAM("linear_y: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;
    msg.linear.y = 1*persentage_of_cmd_vel/100;
    vel_pub.publish(msg);

}

int Drone::sign(double value) {

    return value/std::abs(value);

}


void Drone::linear_z(double persentage_of_cmd_vel) {

    ROS_INFO_STREAM("linear_z: " << persentage_of_cmd_vel << "%");

    vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000, true);
    while(vel_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    geometry_msgs::Twist msg;

    msg.linear.z = persentage_of_cmd_vel/100;

    vel_pub.publish(msg);

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

    //hover();

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


Drone::Euler_angle Drone::to_euler_angle() {

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


    Drone::Euler_angle axis;
    axis.xAxis = roll;
    axis.yAxis = pitch;
    axis.yaw = yaw;

    return axis;

}


Drone::Euler_angle Drone::get_euler_angles() {

    update_odometry();

    Drone::Euler_angle axis;
    axis = to_euler_angle();

    return axis;

}


Drone::Position Drone::get_position() {

    update_odometry();

    Drone::Position position;
    position.x_coord = XCurrentCoord;
    position.y_coord = YCurrentCoord;
    position.z_coord = ZCurrentCoord;

    return position;

}


void Drone::update_odometry() {

    gr_sub = nh.subscribe(odometry_topic, 1000, &Drone::get_odometry, this);
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

}


void Drone::get_odometry(const nav_msgs::Odometry &msg){

    x = msg.pose.pose.orientation.x;
    y = msg.pose.pose.orientation.y;
    z = msg.pose.pose.orientation.z;
    w = msg.pose.pose.orientation.w;

    XCurrentCoord = msg.pose.pose.position.x;
    YCurrentCoord = msg.pose.pose.position.y;
    ZCurrentCoord = msg.pose.pose.position.z;

}


}
