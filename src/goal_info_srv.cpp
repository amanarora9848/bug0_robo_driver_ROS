#include <ros/ros.h>
#include <unige_rt1_assignment2/GoalInfo.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <assignment_2_2022/PlanningAction.h>

// Define the goals service class
class GoalsService {
    private:
    // Define the service server
    ros::ServiceServer goals_service_server;
    int goals_reached = 0, goals_cancelled = 0;
    // ROS::Subscriber listening to /reaching_goal/result for the result of the action;
    ros::Subscriber target_result_subscriber;

    public:
    GoalsService(ros::NodeHandle *n) {
        // Subscribe to /reaching_goal/result topic to read the result of the action with the subscribing queue size of 1
        target_result_subscriber = n->subscribe("/reaching_goal/result", 1, &GoalsService::target_result_callback, this);

        // Create the service server
        goals_service_server = n->advertiseService("/goals_service", &GoalsService::goals_service_callback, this);

        // Handle ROS communication events
        ros::spin();
    }

    void target_result_callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
        if (msg->status.status == 3) {
            goals_reached += 1;
        }
        else if (msg->status.status == 2) {
            goals_cancelled += 1;
        }
    }

    bool goals_service_callback(unige_rt1_assignment2::GoalInfo::Request &req, unige_rt1_assignment2::GoalInfo::Response &res) {
        ROS_INFO("Goal information service request received...");
        // Set the response message
        res.msg_feedback = "\n==========================================\nNumber of goals reached: " + std::to_string(goals_reached) + "\nNumber of goals cancelled: " + std::to_string(goals_cancelled) + "\n==========================================\n";
        return true;
    }
};

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "goal_info_srv_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Create the goals service object
    GoalsService goals_service(&n);

    // exit
    return 0;
}