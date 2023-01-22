#include <ros/ros.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>
#include <map>

class RoboInfo {
    private:
    // Subscriber to robot position and velocity using custom message type
    ros::Subscriber robot_pos_vel_subscriber;
    // Map which saves goal position retrieved from the parameter server
    std::map<std::string, double> goal_map = {
        {"x", 0.0},
        {"y", 0.0}
    };
    // Variables for robot position and velocity
    double pos_x, pos_y, vel_x, vel_z;
    int elapsed_time = 0;

    public:
    RoboInfo(ros::NodeHandle *n, double freq){
        ros::Rate *rrate;

        rrate = new ros::Rate(freq);

        // Check for the goal position parameter on the parameter server
        if (n->hasParam("/robot/goal_pos_param")) {
            // Get the value of the parameter
            n->getParam("/robot/goal_pos_param", goal_map);
        }
        else{
            ROS_ERROR("Goal position parameter not found on the parameter server");
        }

        // Subscribe to custom message topic to read robot positiona nd velocity
        robot_pos_vel_subscriber = n->subscribe("/robo_stats", 10, &RoboInfo::robo_info_callback, this);

        while(ros::ok()) {
            // check for incoming messages
            ros::spinOnce();

            // sleep for rate given by user
            rrate->sleep();
        }
    }

    void robo_info_callback(const unige_rt1_assignment2::RoboStatusMsg::ConstPtr& msg) {
        pos_x = msg->pos_x;
        pos_y = msg->pos_y;
        vel_x = msg->vel_x;
        vel_z = msg->vel_z;
        elapsed_time += 1;
        show_info();
    }

    void show_info() {
        ROS_INFO("Robot Information:");

        // Print robot position and velocity
        ROS_INFO("Robot position: (%f, %f)", pos_x, pos_y);
        ROS_INFO("Robot velocity: (%f, %f)", vel_x, vel_z);
        // Print goal position
        ROS_INFO("Goal position: (%f, %f)", goal_map["x"], goal_map["y"]);
        // Print distance from goal
        ROS_INFO("Distance from goal: %f", distance_from_goal(pos_x, pos_y));
        // Print average speed
        ROS_INFO("Average speed: %f", average_speed());
    }

    double distance_from_goal(double x, double y){
        // calculate distance from goal
        double distance = sqrt(pow(x - goal_map["x"], 2) + pow(y - goal_map["y"], 2));
        return distance;
    }

    double average_speed(){
        // calculate average speed
        double average_sleep = distance_from_goal(pos_x, pos_y) / elapsed_time;
        return 0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robo_info");
    ros::NodeHandle nh;
    
    RoboInfo robo_info(&nh, atof(argv[1]));

    return 0;
}
