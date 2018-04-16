#include "drone.h"

const char cmd_vel_topic[] = "/cmd_vel";
//const char odometry_topic[] = "/ardrone/odometry"; // ARDrone
const char odometry_topic[] = "/ground_truth/state"; // Gazebo


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


int Drone::sign(double value) {

    return value/std::abs(value);

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
