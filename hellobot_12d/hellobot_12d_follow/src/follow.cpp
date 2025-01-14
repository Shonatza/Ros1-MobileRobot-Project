#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <math.h>

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////   MSG     //////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

const char* follow1_msg = R"(
Follow mode
---------------------------

Following reflective object

q: quit
)";

const char* follow2_msg = R"(
Follow mode
---------------------------

Following any object

q: quit
)";


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////////////////// GLOBAL /////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

geometry_msgs::Pose2D current_pose;
int obstacle = 0;
// 0 = no obstacle; 1 = obstacle in front; 2 = obstacle in right; 3 = obstacle in left;

int pt=0; // Print text in stdout
const float max_dist = 0.8;
int rot_time = 0;
float min_int = 10000.0;
float battery_level = 1.0; //Indicates the level of battery from 0.0 to 1.0

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////   AUX FUNCTIONS    //////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

// Function to set terminal to non-blocking mode
void setNonBlockingMode() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);            // Save current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);   // Apply new terminal settings

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // Set non-blocking mode
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
	current_pose.x = msg->pose.pose.position.x;
	current_pose.y = msg->pose.pose.position.y;
	tf::Quaternion q(
	msg->pose.pose.orientation.x,
	msg->pose.pose.orientation.y,
	msg->pose.pose.orientation.z,
	msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	current_pose.theta = yaw;
}

void obs_detect(const sensor_msgs::LaserScan::ConstPtr& msg) {

	obstacle= 0;
	int offset = 80;
	
	for (int i = (offset + 1 - 10); i < (offset + 10); i++){ //go forward
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.1 && msg->intensities[i] > min_int){
			obstacle = 1;
			return;
		}
	}
	
	for (int i = 0; i < (offset + 1 - 10); i++){ //go left
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.1 && msg->intensities[i] > min_int){
			obstacle = 2;
			return;
		}
	}
	
	for (int i = (offset + 10); i < (offset + 180); i++){ //go right
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.1 && msg->intensities[i] > min_int){
			obstacle = 3;
			return;
		}
	}
	
	for (int i = (offset + 180); i < 359; i++){ //go left
		if(msg->ranges[i] < 0.5 && msg->ranges[i] > 0.1 && msg->intensities[i] > min_int){
			obstacle = 2;
			return;
		}
	}
}

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_level = msg->percentage;  // Battery percentage (0.0 to 1.0)
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////   MODE 2 & 3    ///////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

int main(int argc, char **argv) {
	const double PI = 3.14159265358979323846;
	ros::init(argc, argv,"follow");
	ros::NodeHandle n;
	int aux;
	
	system("clear"); // CLEAR CONSOLE
    if(n.getParam("/follow/follow_mode", aux) and aux == 1) {
		min_int = 1.0;
		std::cout << follow2_msg << std::endl;
	}
	else {
		std::cout << follow1_msg << std::endl;
	}

	ros::Subscriber sub_battery = n.subscribe("battery_state", 100, batteryCallback);
	ros::Subscriber sub_odometry = n.subscribe("odom", 100, odomCallback);
	ros::Subscriber sub_scan = n.subscribe("scan", 100, obs_detect);
	ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Rate rate(100);
	geometry_msgs::Twist move;
	geometry_msgs::Pose2D init_pos;

	init_pos.x = current_pose.x;
	init_pos.y = current_pose.y;

	double init_theta = current_pose.theta;
	ROS_INFO("%f %f %f", init_pos.x, init_pos.y, init_theta);

	bool exit = false, battery_msg = false;
	char key;
	setNonBlockingMode();
	while(ros::ok() && not exit) {
		
		if(not battery_msg && battery_level < 0.15) {
			battery_msg = true;
			system("clear"); // CLEAR CONSOLE
			if(aux == 1) {
				std::cout << follow2_msg << std::endl; // FOLLOW MSG MODE 3
			}
			else {
				std::cout << follow1_msg << std::endl; // FOLLOW MSG MODE 2
			}

			std::cout << "Battery level is low" << std::endl;
			ROS_INFO("%f %f %f", init_pos.x, init_pos.y, init_theta);
		}
		
		if (read(STDIN_FILENO, &key, 1) == 1 && key == 'q') {
			exit = true;
			system("clear"); // CLEAR CONSOLE
		} // Check if 'q' is pressed

		if (obstacle==1) { //velocidad angular nula
			move.linear.x = 0.6; 
			pt=1;
			move.angular.z = 0.0;
			};
		if (obstacle==2) { 
			move.linear.x = 0.0; 
			pt=1;
			move.angular.z = -0.6;			//hacer que el robot gire
			};
		if (obstacle==3) { 
			move.linear.x = 0.0; 
			pt=1;
			move.angular.z = 0.6;			//hacer que el robot gire
			};
		if (obstacle==0) {
			move.linear.x = 0.0;
			move.angular.z = 0.0;
			}
		movement_pub.publish(move);
		if (obstacle==0 && pt==1) {
			ROS_INFO("%d", obstacle);
			ROS_INFO("X: %f Y: %f Theta: %f", current_pose.x, current_pose.y,
			current_pose.theta);
			pt=0;
		};
		ros::spinOnce();
		rate.sleep();
	};

	move.linear.x = 0.0;
	move.angular.z = 0.0;
	return 0;
}