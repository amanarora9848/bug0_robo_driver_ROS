// Include ros stuff
#include <ros/ros.h>
// Include the service file
#include <unige_rt1_assignment2/GoalInfo.h>

// Define the goals service class
class GoalsService {
    private:
    // Define the service server
    ros::ServiceServer goals_service_server;
    int goals_reached = 0, goals_cancelled = 0;

    public:
    GoalsService(ros::NodeHandle *n) {
        if (n->hasParam("/robot/reached_goals")) {
            n->getParam("/robot/reached_goals", goals_reached);
        }
        else{
            ROS_ERROR("Reached goals parameter not found on the parameter server");
        }
        if (n->hasParam("/robot/cancelled_goals")) {
            n->getParam("/robot/cancelled_goals", goals_cancelled);
        }
        else{
            ROS_ERROR("Cancelled goals parameter not found on the parameter server");
        }
        // Create the service server
        goals_service_server = n->advertiseService("/goals_service", &GoalsService::goals_service_callback, this);
    }

    bool goals_service_callback(unige_rt1_assignment2::GoalInfo::Request &req, unige_rt1_assignment2::GoalInfo::Response &res) {
        ROS_INFO("Goal information service request received...");
        
        // Set the response message
        res.msg_feedback = "Number of goals reached: " + std::to_string(goals_reached) + "\nNumber of goals cancelled: " + std::to_string(goals_cancelled);
        ROS_INFO_STREAM(res.msg_feedback);
        // Return true
        return true;
    }
};

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "goal_info_srv");
    // Create the node handler
    ros::NodeHandle n;
    // Create the goals service object
    GoalsService goals_service(&n);
    // Spin
    ros::spin();
    return 0;
}