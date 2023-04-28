/**
 * \file goal_info_srv.cpp
 * \author Aman Arora (aman.arora9848@gmail.com)
 * \brief Goal information service node.
 * \version 1.0
 * \date 15/03/2023
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *  ° /reaching_goal/result
 * 
 * Services: <BR>
 * ° /goals_service
 * 
 * Description:
 * 
 * This node recieves the result of the action done by the user to send a goal
 * and obtains this info from the topic /reaching_goal/result.
 * It also provides a service /goals_service to provide the number of goals reached and cancelled.
 * 
 */

#include <ros/ros.h>
#include <unige_rt1_assignment2/GoalInfo.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <assignment_2_2022/PlanningAction.h>

/**
 * \class GoalsService
 * 
 * \brief Goals service class.
 * 
 * This class is used to create a node that subscribes to the topic /reaching_goal/result and 
 * obtains the result of the action done by the user to send a goal.
 * It also provides a service /goals_service to provide the number of goals reached and cancelled.
 * 
 * Member functions:
 *  - GoalsService(ros::NodeHandle *n)
 *   + This is the constructor of the class.
 *  - target_result_callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg)
 *   + This is the callback function for the subscriber.
 *  - goals_service_callback(unige_rt1_assignment2::GoalInfo::Request &req, unige_rt1_assignment2::GoalInfo::Response &res)
 *   + This is the callback function for the service advertiser.
 * 
 */
class GoalsService {
    private:
    ros::ServiceServer goals_service_server; /**< Service server for goals service */
    int goals_reached = 0, goals_cancelled = 0; /**< Variables for goals reached and cancelled */
    ros::Subscriber target_result_subscriber; /**< Subscriber for the topic /reaching_goal/result for the result of the action */

    public:
    GoalsService(ros::NodeHandle *n) {
        /**
         * \brief Constructor for the class.
         * 
         * \param n Pointer to the node handle.
         * 
         * \return None
         * 
         * This constructor creates a subscriber for the topic /reaching_goal/result and 
         * a service server for the service /goals_service.
         * 
         */

        /**
         * \brief target_result_subscriber
         * 
         * This variable is a subscriber for the topic /reaching_goal/result for the result of the action.
         * 
         */
        target_result_subscriber = n->subscribe("/reaching_goal/result", 1, &GoalsService::target_result_callback, this);

        /** 
         * \brief goals_service_server
         * 
         * This variable is a service server for the service /goals_service.
         * 
         */
        goals_service_server = n->advertiseService("/goals_service", &GoalsService::goals_service_callback, this);

        ros::spin();
    }

    // This function gets called whenever a new message arrives on the /reaching_goal/result topic
    void target_result_callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
        /**
         * \brief Callback function for the subscriber.
         * 
         * \param msg The message of type PlanningActionResult received on the topic.
         * 
         * \return None
         * 
         * This function gets called whenever a new message arrives on the /reaching_goal/result topic.
         * It updates the variables goals_reached and goals_cancelled according to the status of the message.
         * 
         */
        if (msg->status.status == 3) {
            goals_reached += 1;
        }
        else if (msg->status.status == 2) {
            goals_cancelled += 1;
        }
    }

    // This function gets called whenever a new service request arrives on the /goals_service topic
    bool goals_service_callback(unige_rt1_assignment2::GoalInfo::Request &req, unige_rt1_assignment2::GoalInfo::Response &res) {
        /**
         * \brief Callback function for the service advertiser.
         * 
         * \param req The request message of type GoalInfo received on the service.
         * \param res The response message of type GoalInfo to be sent on the service.
         * 
         * \return bool
         * 
         * This function gets called whenever a new service request arrives on the /goals_service topic.
         * It sets the response message and returns true.
         * 
         */
        ROS_INFO("Goal information service request received...");
        /**
         * \brief res.msg_feedback
         * 
         * This variable is the response message of type GoalInfo to be sent on the service.
         * 
         */
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