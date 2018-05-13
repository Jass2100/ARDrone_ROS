#include "drone.h"
#include <cstdlib>
#include <stdbool.h>

using namespace std;

int sign(double value);
void test_move2point(double goal_x , double goal_y);
void test_2_move2point(double goal_x_coordinate, double goal_y_coordinate);
double start_time = 0;

int main(int argc, char **argv){

    ros::init(argc, argv, "drone_node");

    return 0;

}

int sign(double value) {

    return value/std::abs(value);

}


void test_move2point(double goal_x , double goal_y) {

    double bearing = 0, course_angle = 0;
    double linear_speed = 0, angular_speed = 0;
    double length = 0;
    double linear_max_speed = 30, angular_max_speed = 300;
    double ERROR = 0.1;

    double kp_angular = 10;

    Drone drone;

    // есди набежал угол, то необъодимо его обнулить
    double course = drone.get_euler_angles().yaw;

    int state_of_drone = 0;
    state_of_drone = drone.get_navdata().state;
    if (state_of_drone != 3) {

        drone.take_off();
        sleep(6);

    }
    else {

        cout << "drone has already taken off" << endl;

    }

    // если наш квадрокоптер стартует не из начала, то надо обнулить координаты
    double current_x_coord = drone.get_position().x_coord;
    double current_y_coord = drone.get_position().y_coord;

    //cout << "course: " << course << " | x: " << current_x_coord << " | y: " << current_y_coord << endl;

    while(1) {

        course = drone.get_euler_angles().yaw;

        // ищем ошибку по оси x 
        current_x_coord = drone.get_position().x_coord;
        double deltha_x = goal_x - current_x_coord;

        // ищем ошибку по оси y
        current_y_coord = drone.get_position().y_coord;
        double deltha_y = goal_y - current_y_coord;

        // расчёт пеленга
        bearing = atan2(deltha_y, deltha_x)*180/3.1415;

        // расчёт курсового угла
        course_angle = bearing - course;

        // разворот по кротчайшему пути
        if (abs(course_angle) > 180) course_angle = course_angle - sign(course_angle) * 360;

        //cout << bearing << " | " << course_angle << " | " << deltha_x << " | " << deltha_y << endl;

        // расчёт линейной скорости
        length = sqrt(deltha_y * deltha_y + deltha_x * deltha_x);
        linear_speed = abs(100 * tanh (length) * cos (course_angle));
        if (abs(linear_speed) > linear_max_speed) linear_speed = sign(linear_speed) * linear_max_speed;

        //cout << length << " | " << linear_speed << endl;

        // расчёт угловой скорости
        angular_speed = kp_angular * course_angle + sin(course_angle) * linear_speed / length;
        if (abs(angular_speed) > angular_max_speed) angular_speed = sign(angular_speed) * angular_max_speed;

        // подача управления
        drone.test_reach_point(linear_speed, angular_speed);

        //cout << deltha_x << " | " << deltha_y << endl;
        cout << current_x_coord << " " << current_y_coord << " " << ros::Time::now().toSec() - start_time << endl;

        if(abs(length) < ERROR) break;

    }

}

void test_2_move2point(double goal_x_coordinate, double goal_y_coordinate) {

    double current_yaw = 0, goal_yaw = 0, phi = 0, theta = 0;
    double current_x_coordinate = 0, current_y_coordinate = 0;
    double yaw_offset = 0, x_coordinate_offset = 0, y_coordinate_offset = 0;
    double distance_error = 0;
    double rad2deg = 180/3.1415, deg2rad = 1/rad2deg;
    double max_velocity = 1;
    double x_linear_velocity_component = 0, y_linear_velocity_component = 0;
    double coefficient_of_saturation_proportionality = 1;
    bool in_a_neighborhood_of_the_point = false;
    double error_of_goal_achievement = 0.1;

    // initialization drone control class
    Drone drone;

    start_time = ros::Time::now().toSec();

    // take off drone and wait to drone hover
    int state_of_drone = 0;
    state_of_drone = drone.get_navdata().state;
    if (state_of_drone != 3) {

        drone.take_off();
        sleep(6);

    }
    else {

        cout << "drone has already taken off" << endl;

    }

    coordinates and yaw angle calibration( get offset)
    cout << "" << endl;
    cout << "cordinates and yaw calibration..." << endl;
    cout << "__________________________________" << endl;
    cout << "" << endl;
    yaw_offset = drone.get_euler_angles().yaw;
    x_coordinate_offset = drone.get_position().x_coord;
    y_coordinate_offset = drone.get_position().y_coord;
    cout << "yaw offset: " << yaw_offset << endl;
    cout << "x offset: " << x_coordinate_offset << endl; 
    cout << "y offset: " << y_coordinate_offset << endl;
    cout << "__________________________________" << endl;
    cout << "" << endl;

    while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate << " " << current_y_coordinate << " " << ros::Time::now().toSec() - start_time  << endl;

    }

        double a = current_x_coordinate;
        double b = current_y_coordinate;

        in_a_neighborhood_of_the_point = false;
        yaw_offset = drone.get_euler_angles().yaw;
        x_coordinate_offset = drone.get_position().x_coord;
        y_coordinate_offset = drone.get_position().y_coord;

        goal_x_coordinate = 0;
        goal_y_coordinate = -1;

        while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate + a << " " << current_y_coordinate + b << " " << ros::Time::now().toSec() - start_time << endl;

    }

        a = a + current_x_coordinate;
        b = b + current_y_coordinate;

        in_a_neighborhood_of_the_point = false;
        yaw_offset = drone.get_euler_angles().yaw;
        x_coordinate_offset = drone.get_position().x_coord;
        y_coordinate_offset = drone.get_position().y_coord;

        goal_x_coordinate = -1;
        goal_y_coordinate = 0;

        while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate + a << " " << current_y_coordinate + b << " " << ros::Time::now().toSec() - start_time  << endl;

    }

        a = a + current_x_coordinate;
        b = b + current_y_coordinate;

        in_a_neighborhood_of_the_point = false;
        yaw_offset = drone.get_euler_angles().yaw;
        x_coordinate_offset = drone.get_position().x_coord;
        y_coordinate_offset = drone.get_position().y_coord;

        goal_x_coordinate = 0;
        goal_y_coordinate = 1;

        while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate + a << " " << current_y_coordinate + b << " " << ros::Time::now().toSec() - start_time  << endl;

    }


        a = a + current_x_coordinate;
        b = b + current_y_coordinate;

        in_a_neighborhood_of_the_point = false;
        yaw_offset = drone.get_euler_angles().yaw;
        x_coordinate_offset = drone.get_position().x_coord;
        y_coordinate_offset = drone.get_position().y_coord;

        goal_x_coordinate = 1;
        goal_y_coordinate = 0;

        while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate + a << " " << current_y_coordinate + b << " " << ros::Time::now().toSec() - start_time  << endl;

    }


        a = a + current_x_coordinate;
        b = b + current_y_coordinate;

        in_a_neighborhood_of_the_point = false;
        yaw_offset = drone.get_euler_angles().yaw;
        x_coordinate_offset = drone.get_position().x_coord;
        y_coordinate_offset = drone.get_position().y_coord;

        goal_x_coordinate = -1;
        goal_y_coordinate = -1;

        while(!in_a_neighborhood_of_the_point) {

        // update quadrotor navdata
        current_x_coordinate = drone.get_position().x_coord - x_coordinate_offset;
        current_y_coordinate =  drone.get_position().y_coord - y_coordinate_offset;
        current_yaw = drone.get_euler_angles().yaw - yaw_offset;

        // calculation dustance error
        double delta_x = goal_x_coordinate - current_x_coordinate;
        double delta_y = goal_y_coordinate - current_y_coordinate;
        distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);

        // calculation of signal distribution
        theta = atan2(delta_y, delta_x) * rad2deg;
        phi = theta - current_yaw;
        x_linear_velocity_component = max_velocity * cos(phi * deg2rad);
        y_linear_velocity_component = max_velocity * cos((phi - 90) * deg2rad);

        // calculation of signal saturation
        double x_signal_saturation = x_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        double y_signal_saturation = y_linear_velocity_component * distance_error * coefficient_of_saturation_proportionality;
        if (abs(x_signal_saturation) > 1) x_signal_saturation = max_velocity * sign(x_signal_saturation);
        if (abs(y_signal_saturation) > 1) y_signal_saturation = max_velocity * sign(y_signal_saturation);

        // set velocity
        drone.xy_linear_velocity_control(x_signal_saturation, y_signal_saturation);

        if (distance_error < error_of_goal_achievement) in_a_neighborhood_of_the_point = true;

        cout << current_x_coordinate + a << " " << current_y_coordinate + b << " " << ros::Time::now().toSec() - start_time  << endl;

    }

    //cout << "reach the destination x: " << goal_x_coordinate << "; y: " << goal_y_coordinate << endl;

}
