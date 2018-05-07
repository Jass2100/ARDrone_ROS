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
    //Drone dr;
    //start_time = ros::Time::now().toSec();
    //test_move2point(1, 1);
    //test_move2point(1, 0);
    //test_move2point(0, 0);
    //test_move2point(0, 1);
    //test_move2point(1, 1);
    //test_move2point(0, 0);
    test_2_move2point(1, 1);
    //test_2_move2point(0, -1);
    //test_2_move2point(-1, 0);
    //test_2_move2point(0, 1);
    //test_2_move2point(1, 0);
    //test_2_move2point(-1, -1);

    //Drone drone;
    //drone.calculate_x_coordinations();

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

        //cout << "drone has already taken off" << endl;

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
