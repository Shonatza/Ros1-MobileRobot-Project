#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////   MSG     //////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

const char* free_msg = R"(
Free mode
---------------------------

        w
   a    s    d
        x

w: increase linear velocity (FORWARD)
x: decrease linear velocity (BACKWARD)
a: increase angular velocity (LEFT)
d: decrease angular velocity (RIGHT)
s: stop
space: move with the current velocity

q: quit
)";

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////////////////// GLOBAL /////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

float battery_level = 1.0; //Indicates the level of battery from 0.0 to 1.0

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////   AUX FUNCTIONS    //////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

char get_key() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

double make_simple_profile(double output, double input, double slop) {
    if (input > output) {
        output = std::min(input, output + slop);
    } else if (input < output) {
        output = std::max(input, output - slop);
    } else {
        output = input;
    }
    return output;
}

double constrain(double input_vel, double low_bound, double high_bound) {
    if (input_vel < low_bound) {
        input_vel = low_bound;
    } else if (input_vel > high_bound) {
        input_vel = high_bound;
    }
    return input_vel;
}

double check_linear_limit_velocity(double velocity) {

    return constrain(velocity, -0.22, 0.22);

}

double check_angular_limit_velocity(double velocity) {

    return constrain(velocity, -2.84, 2.84);

}

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_level = msg->percentage;  // Battery percentage (0.0 to 1.0)
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////////   MODE 1    /////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

int main(int argc, char** argv) {

    ros::init(argc, argv,"free"); // INITIALIZE

	ros::NodeHandle node; // INITIALIZE VARIABLES
    ros::Subscriber sub_battery = node.subscribe("battery_state", 100, batteryCallback);
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    double target_linear_velocity = 0.0;
    double target_angular_velocity = 0.0;
    double control_linear_velocity = 0.0;
    double control_angular_velocity = 0.0;

    system("clear"); // CLEAR CONSOLE
	std::cout << free_msg << std::endl; // FREE MSG
    std::cout << "Linear Velocity: " << target_linear_velocity << ", Angular Velocity: " << target_angular_velocity << std::endl; // Velocities

    bool exit = false, battery_msg = false;
    char key;
    try {
        while (ros::ok() && not exit) { // KEY CHOICE
            key = get_key();
            if (key == 'w') {
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + 0.01);
            } else if (key == 'x') {
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - 0.01);
            } else if (key == 'a') {
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + 0.1);
            } else if (key == 'd') {
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - 0.1);
            } else if (key == 's') {
                target_linear_velocity = 0.0;
                control_linear_velocity = 0.0;
                target_angular_velocity = 0.0;
                control_angular_velocity = 0.0;
            } else if(key == 'q') {
				exit = true;
            }

            geometry_msgs::Twist twist; // UPDATE VELOCITIES

            control_linear_velocity = make_simple_profile(control_linear_velocity, target_linear_velocity, 0.005);

            twist.linear.x = control_linear_velocity;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            control_angular_velocity = make_simple_profile(control_angular_velocity, target_angular_velocity, 0.05);

            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = control_angular_velocity;

            pub.publish(twist);

            system("clear"); // CLEAR CONSOLE
            std::cout << free_msg << std::endl; // FREE MSG
            if(battery_level < 0.15) {
                std::cout << "Battery level is low" << std::endl;
            }
            std::cout << "Linear Velocity: " << target_linear_velocity << ", Angular Velocity: " << target_angular_velocity << std::endl; // Velocities
            ros::Rate rate(10);

        }
    } catch (const std::exception& e) { // ERROR
        std::cerr << e.what() << std::endl;
    }

    geometry_msgs::Twist twist; // STOP
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    pub.publish(twist);
}