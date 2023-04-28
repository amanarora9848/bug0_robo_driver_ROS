/** 
 * \file target_info_client.cpp
 * \author Aman Arora (aman.arora9848@gmail.com)
 * \brief Controller node for the robot to move to a target position.
 *        Implements an action client, allowing the user to set a target position (x, y), or cancel it.
 * \version 1.0
 * \date 15/03/2023
 * 
 * \param[in] /robot/goal_pos_param/x_goal The x position of the goal
 * \param[in] /robot/goal_pos_param/y_goal The y position of the goal 
 * 
 * \details
 *  
 * Actions: <BR>
 *   ° /reaching_goal
 * 
 * Description:
 * 
 * This node is used to create a client node that sends goals to the action server defined in package assignment_2_2022.
 * It interacts wuth the action serverusing the Planning.action file defined in the package "assignment_2_2022".
 * 
 * Usage Instructions:
 * - After launching the simulation using the launch file:
 *   + Enter 'p' and press [ ENTER ] to input a goal position (x, y) for the robot to move to.
 *   + Enter 'q' and press [ ENTER ] to cancel the current goal.
 * 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <stack>


/**
 * \class RoboClient
 *       
 * \brief Implements an action client, allowing the user to set a target position (x, y), or cancel it.
 *
 *  This class is used to create a client node that sends goals to the action server defined in package assignment_2_2022.
 *  The user can input a goal position after entering 'p' and pressing [ ENTER ] and the robot will move to that position.
 *  The user can also cancel the current goal by entering 'q'.
 * 
 *  The class contains the following functions:
 *  - RoboClient(ros::NodeHandle *n)
 *    + This is the constructor of the class.
 * 
 *  The class contains the following variables:
 *  - double x, y: The x and y position of the goal
 *  - std::string goal_input: The user input for the goal command
 *  - std::stack<char> goal_stack: The stack to store the user commands
 * 
 *  The class contains the following actionlib client:
 *  - actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> target_ac
 *   + This is the actionlib client that sends goals to the action server.
 * 
 */
class RoboClient {
    private:
    double x, y; /**< Declare x and y position to be retrieved as user input */
    std::string goal_input; /**< Declare input string for user command */
    std::stack<char> goal_stack; /**< Declare stack to store user commands */

    public:
    RoboClient(ros::NodeHandle *n) {
        /**
         * \brief Constructor for RoboClient class.
         * 
         * \param n: The node handle object.
         * \return None
         * 
         * This constructor is used to create the action client, and send goals to the action server. It also lets the user set a goal position (x, y) or cancel the current goal, which is given to the action server for execution and simulation.
         * 
         */

        /** create the action client, true causes the client to spin its own thread */
        actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> target_ac("/reaching_goal", true);

        ROS_INFO("Waiting for action server to start.");
        target_ac.waitForServer(); /**< Wait for the action server to start, will wait for infinite time */

        ROS_INFO("Action server started! Press [ ENTER ] to request a goal.");

        std::cout << "\n==========================================\n" << std::endl;

        /** send the initial goal to the action server */
        assignment_2_2022::PlanningGoal goal; /**< Declare goal object */

        while(ros::ok()) {
            std::cout << "Enter [ p ] to input required goal position, or enter [ q ] to cancel the current set goal: ";
            std::cin >> goal_input;
            /** Check if user input is p or q */
            if (goal_input == "p") {
                /** Check if q was previously entered and remove it from the stack */
                if (!goal_stack.empty()) {
                    goal_stack.pop();
                }
                std::cout << "Enter required goal position x y: "; /**< Get x and y position from user */
                std::cin >> x >> y;
                /** Send goal to action server */
                goal.target_pose.pose.position.x = x; /**< Set x position in goal object */
                goal.target_pose.pose.position.y = y; /**< Set y position in goal object */
                target_ac.sendGoal(goal);
                /** Set the goal positions in the parameter server variables */
                if (n->hasParam("/robot/goal_pos_param")) {
                    n->setParam("/robot/goal_pos_param/x_goal", x); /**< Set x position in parameter server */
                    n->setParam("/robot/goal_pos_param/y_goal", y); /**< Set y position in parameter server */
                }
                else {
                    ROS_ERROR("Goal position parameter not found on the parameter server."); /**< Error message if parameter not found */
                }
            }
            else if (goal_input == "q") {
                /** Ensure that q has not already been pressed by user */
                if (goal_stack.empty()) {
                    goal_stack.push('q');
                    /** Check if there is a goal currently executing */
                    if (target_ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                        std::cout << "Cancelling current goal..." << std::endl;
                        target_ac.cancelGoal();
                    }
                    else{
                        std::cout << "No goal is currently executing." << std::endl;
                    }
                }
                else if (goal_stack.top() == 'q') {
                    std::cout << "No goal is currently executing." << std::endl;
                }
                else {
                    std::cout << "Invalid input. Please press [ ENTER ] to input x and y values." << std::endl;
                }
            }
            else {
                std::cout << "Invalid input. Please enter [ ENTER ] or 'q' again." << std::endl;
            }

            std::cout << "\n==========================================\n" << std::endl;

            /** Check for new messages from the subscribed topics */
            ros::spinOnce();
        }
    }
};

int main (int argc, char **argv) {
    // Initialize the robot_nav_client node
    ros::init(argc, argv, "target_info_client_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Create the robot action client object
    RoboClient rc(&n);

    //exit
    return 0;
}