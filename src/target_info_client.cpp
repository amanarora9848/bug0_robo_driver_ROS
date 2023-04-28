#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <stack>

class RoboClient {
    private:
    // Declare x and y position to be retrieved as user input
    double x, y;
    // Declare input string for user command
    std::string goal_input;
    // Declare stack to store user commands
    std::stack<char> goal_stack;

    public:
    RoboClient(ros::NodeHandle *n) {
        // create the action client, true causes the client to spin its own thread
        actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> target_ac("/reaching_goal", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        target_ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started! Press [ ENTER ] to request a goal.");

        std::cout << "\n==========================================\n" << std::endl;

        // send the initial goal to the action
        assignment_2_2022::PlanningGoal goal;

        while(ros::ok()) {
            std::cout << "Enter [ p ] to input required goal position, or enter [ q ] to cancel the current set goal: ";
            std::cin >> goal_input;
            // Check if user input is p or q
            if (goal_input == "p") {
                // Check if q was previously entered and remove it from the stack
                if (!goal_stack.empty()) {
                    goal_stack.pop();
                }
                // Get x and y position from user
                std::cout << "Enter required goal position x y: ";
                std::cin >> x >> y;
                // Send goal to action server
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                target_ac.sendGoal(goal);
                // Set the goal positions in the parameter server variables
                if (n->hasParam("/robot/goal_pos_param")) {
                    n->setParam("/robot/goal_pos_param/x_goal", x);
                    n->setParam("/robot/goal_pos_param/y_goal", y);
                }
                else {
                    ROS_ERROR("Goal position parameter not found on the parameter server.");
                }
            }
            else if (goal_input == "q") {
                // Ensure that q has not already been pressed by user
                if (goal_stack.empty()) {
                    goal_stack.push('q');
                    // Check if there is a goal currently executing
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

            // Check for new messages from the subscribed topics
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