#include <cstdlib>
#include <iostream>
#include <sensor_msgs/BatteryState.h>
#include <ros/ros.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>   // For fork(), exec() on Linux
#include <sys/types.h> // For pid_t, process-related structures
#include <sys/wait.h>  // For wait() to handle child processes
#include <signal.h>    // For signal constants (e.g., SIGKILL)
#include <errno.h>     // For errno

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////   MSG     //////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

const char* msg = R"(
Choose a mode
---------------------------
1: free mode
2: follow reflective object
3: follow any object
4: turn on SLAM

q: quit
)";

const char* msg2 = R"(
Choose a SLAM mode
---------------------------
1: free mode

q: quit (you should close rviz manually)
)";

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////////////////// GLOBAL /////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

float battery_level = 1.0; //Indicates the level of battery from 0.0 to 1.0

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
/////////////////   AUX FUNCTION     //////////////////
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

// Function to reset terminal to blocking mode
void resetTerminalMode(struct termios &saved_termios) {
    // Restore terminal settings to original state
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK); // Set non-blocking mode
}

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    battery_level = msg->percentage;  // Battery percentage (0.0 to 1.0)
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////   MAIN    //////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

int main(int argc, char** argv) {
    ros::init(argc, argv,"main"); // INITIALIZE
	ros::NodeHandle node; // INITIALIZE VARIABLES
    ros::Subscriber sub_battery = node.subscribe("battery_state", 100, batteryCallback); // SUBSCRIBE TO BATTERY STATE  
    struct termios saved_termios;
    tcgetattr(STDIN_FILENO, &saved_termios); // Save current terminal settings (Linux/macOS)
    bool exit = false;
    try {
        while(not exit) { // KEY CHOICE
            resetTerminalMode(saved_termios); // RESET TERMINAL
            system("clear"); // CLEAR CONSOLE
            if(battery_level < 0.15) {
                std::cout << "Battery level is low" << std::endl;
                std::cout << "You must quit the program and change the battery" << std::endl << std::endl;
                std::cout << "q: quit" << std::endl;
                while(key != q) char key = get_key() // WAIT
                exit = true;
            }
            std::cout << msg << std::endl; // MSG
	        char key = get_key();
            if(key == '1') system("roslaunch hellobot_12d_free hellobot_12d_free_launch_file.launch");
            else if(key == '2') system("roslaunch hellobot_12d_follow hellobot_12d_follow_launch_file.launch");
            else if(key == '3') system("roslaunch hellobot_12d_follow hellobot_12d_follow_launch_file.launch follow_mode:=1");
            else if(key == '4') {
                pid_t pid_slam = fork(); // FORK
                if(pid_slam == -1) throw std::runtime_error("fork() failed"); //ERROR
                if(pid_slam == 0) { // CHILD
                    system("roslaunch turtlebot3_slam turtlebot3_slam.launch > /dev/null 2>&1 &");
                    return 0; // EXIT
                }
                else { // PARENT
                    int status;
                    while(not exit) { // CHECK IF PROCESS IS RUNNING
                        system("clear"); // CLEAR CONSOLE
                        std::cout << msg2<< std::endl;
                        char key = get_key();
                        if(key == '1') system("roslaunch hellobot_12d_free hellobot_12d_free_launch_file.launch");
                        else if(key == 'q') exit = true; // KILL
                    }
                    exit = false;
                    system("clear"); // CLEAR CONSOLE
                }
            }
            else if (key == 'q') exit = true;
        }
    } catch (const std::exception& e) { // ERROR
        std::cerr << e.what() << std::endl;
    }

    system("clear"); // CLEAR CONSOLE
    return 0;
}
