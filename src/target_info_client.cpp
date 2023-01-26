#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>
#include <nav_msgs/Odometry.h>


class RoboClient {
    private:
    // Declare x and y position to be retrieved as user input
    double x, y;
    // Declare input string for user command
    std::string goal_input;

    public:
    RoboClient(ros::NodeHandle *n)
    {
        // create the action client, true causes the client to spin its own thread
        actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> target_ac("/reaching_goal", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        target_ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        std::cout << std::endl;

        // send the initial goal to the action
        assignment_2_2022::PlanningGoal goal;

        while(ros::ok()) {
            std::cout << "Enter [ c ] to cancel the current goal (if still in execution), or [ g ] to input required goal position: ";
            std::cin >> goal_input;
            if (goal_input == "g") {
                // Get x and y position from user
                std::cout << "Enter required goal position x y: ";
                std::cin >> x >> y;
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                target_ac.sendGoal(goal);
                // Set the goal positions in the parameter server variables
                if (n->hasParam("/robot/goal_pos_param/x_goal") && n->hasParam("/robot/goal_pos_param/y_goal")) {
                    n->setParam("/robot/goal_pos_param/x_goal", x);
                    n->setParam("/robot/goal_pos_param/y_goal", y);
                }
                else {
                    ROS_ERROR("Goal position parameter not found on the parameter server");
                }
            }
            else if (goal_input == "c") {
                std::cout << "Cancelling..." << std::endl;
                target_ac.cancelGoal();
            }
            else {
                std::cout << "Invalid input. Please enter either 'g' or 'c' again." << std::endl;
            }

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

    RoboClient rc(&n);

    //exit
    return 0;
}