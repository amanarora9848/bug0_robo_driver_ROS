#include <ros/ros.h>
#include <unige_rt1_assignment2/GoalInfo.h>

class GoalInfoClient {
    private:
    // Define the service client
    ros::ServiceClient goal_info_client;
    // Define the service message
    unige_rt1_assignment2::GoalInfo goal_info_srv;

    public:
    GoalInfoClient(ros::NodeHandle *n, double freq) {
        // Set the frequency of the loop
        ros::Rate *loop_rate;
        loop_rate = new ros::Rate(freq);

        // Create the service client
        goal_info_client = n->serviceClient<unige_rt1_assignment2::GoalInfo>("/goals_service");
        
        while(ros::ok()) {
            // Call the service
            if (goal_info_client.call(goal_info_srv)) {
                ROS_INFO_STREAM(goal_info_srv.response.msg_feedback);
            }
            else {
                ROS_ERROR("Failed to call service /goals_service");
            }
            // check for incoming messages
            ros::spinOnce();
            // sleep for the time remaining
            loop_rate->sleep();
        }

        delete loop_rate;
    }
};

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "disp_goals_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Create the goal info client object
    GoalInfoClient goal_info_client(&n, atof(argv[1]));
    
    // exit
    return 0;
}