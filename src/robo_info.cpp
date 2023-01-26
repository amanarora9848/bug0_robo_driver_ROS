#include <ros/ros.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>

class RoboInfo {
    private:
    // Subscriber to robot position and velocity using custom message type
    ros::Subscriber robot_pos_vel_subscriber;
    // Goal position, total velocities and array for average speeds
    double x_goal = 0.0, y_goal = 0.0, vx_tot = 0, vy_tot = 0;
    std::array<double, 3> average_speeds;

    // Variables for robot position and velocity
    double pos_x, pos_y, vel_x, vel_y;
    int elapsed_time = 0;

    public:
    RoboInfo(ros::NodeHandle *n, double freq) {
        ros::Rate *loop_rate;

        loop_rate = new ros::Rate(freq);

        // Subscribe to custom message topic to read robot positiona nd velocity
        robot_pos_vel_subscriber = n->subscribe("/robot/robo_stats", 1, &RoboInfo::robo_info_callback, this);

        while(ros::ok()) {
            // Check for the goal position parameter on the parameter server
            if (n->hasParam("/robot/goal_pos_param")) {
                // Get the value of the parameter
                n->getParam("/robot/goal_pos_param/x_goal", x_goal);
                n->getParam("/robot/goal_pos_param/y_goal", y_goal);
            }
            else{
                ROS_ERROR("Goal position parameter not found on the parameter server");
            }

            // check for incoming messages
            ros::spinOnce();

            // sleep for rate given by user
            loop_rate->sleep();
        }

        delete loop_rate;

    }

    void robo_info_callback(const unige_rt1_assignment2::RoboStatusMsg::ConstPtr& msg) {
        pos_x = msg->pos_x;
        pos_y = msg->pos_y;
        vel_x = msg->vel_x;
        vel_y = msg->vel_y;
        vx_tot += std::abs(vel_x);
        vy_tot += std::abs(vel_y);
        elapsed_time += 1;
        show_info();
    }

    void show_info() {
        // Print robot position and velocity
        std::cout << "\nRobot Information: " << "\n==========================================\n" << std::endl;
        ROS_INFO("Robot position (x, y): (%f, %f)", pos_x, pos_y);
        ROS_INFO("Robot velocity (x, y): (%f, %f)", vel_x, vel_y);
        // Print goal position
        ROS_INFO("Goal position (x, y): (%f, %f)", x_goal, y_goal);
        // Print distance from goal
        ROS_INFO("Distance from goal: %f", distance_from_goal(pos_x, pos_y));
        // Print average speed
        average_speed();
        ROS_INFO("Average speed in x: %f", average_speeds[0]);
        ROS_INFO("Average speed in y: %f", average_speeds[1]);
        ROS_INFO("Average speed overall: %f", average_speeds[2]);
        std::cout << "\n==========================================\n" << std::endl;
    }

    double distance_from_goal(double x, double y){
        // calculate distance from goal
        double distance = sqrt(pow(x - x_goal, 2) + pow(y - y_goal, 2));
        return distance;
    }

    void average_speed(){
        // calculate average speed
        double average_speed_x =  vx_tot / elapsed_time;
        double average_speed_y =  vy_tot / elapsed_time;
        double average_speed = std::sqrt(std::pow(average_speed_x, 2) + std::pow(average_speed_y, 2));
        average_speeds = {average_speed_x, average_speed_y, average_speed};
    }
};

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "robo_info_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle nh;
    
    // Create the robot info object
    RoboInfo robo_info(&nh, atof(argv[1]));

    // exit
    return 0;
}
