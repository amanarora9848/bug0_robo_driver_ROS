#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>
#include <nav_msgs/Odometry.h>
#include <map>


class RoboClient {
    private:
    // Define variables for position and velocity to be published
    float pos_x, pos_y, vel_x, vel_z;
    // ROS::Subscriber listening to odom for robot position and velocity;
    ros::Subscriber robot_pos_vel_subscriber;
    // ROS::Publisher for robot position and velocity;
    ros::Publisher robot_pos_vel_publisher;
    
    public:
    RoboClient(ros::NodeHandle *n, double freq)
    {
        // Subscribe to /odom topic to read the position and velocity of the robot with the subscribing queue size of 10
        robot_pos_vel_subscriber = n->subscribe("/odom", 10, &RoboClient::odom_callback, this);
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        robot_pos_vel_publisher = n->advertise<unige_rt1_assignment2::RoboStatusMsg>("/robot/robo_stats", 10);

        // Publish the robot position and velocity on the robot_pos_vel_publisher Publisher
        unige_rt1_assignment2::RoboStatusMsg robo_status;

        // create the action client, true causes the client to spin its own thread
        actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> target_ac("/reaching_goal", true);

        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        target_ac.waitForServer(); //will wait for infinite time

        ROS_INFO("Action server started, sending goal.");

        std::cout << std::endl;

        // // Create a target pose
        // geometry_msgs::PoseStamped set_pose;
        // set_pose.pose.position.x = 1.0;
        // set_pose.pose.position.y = 1.0;

        // Get x and y position goal as user input in real-time
        std::string goal_input;
        double x, y;
        std::map<std::string, double> goal_map;
        ros::Rate *rrate;

        rrate = new ros::Rate(freq);

        // Set x and y value for goal position.
        x = 2.0;
        y = -5.0;

        // Setting parameter for goal position for node C
        goal_map["x"] = x;
        goal_map["y"] = y;
        n->setParam("/robot/goal_pos_param", goal_map);

        // send the initial goal to the action
        assignment_2_2022::PlanningGoal goal;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        target_ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = target_ac.waitForResult(rrate->expectedCycleTime());

        while(!finished_before_timeout && ros::ok()) {

            // Publish the robot position and velocity on the robot_pos_vel_publisher Publisher
            unige_rt1_assignment2::RoboStatusMsg robo_status;
            robo_status.pos_x = pos_x;
            robo_status.pos_y = pos_y;
            robo_status.vel_x = vel_x;
            robo_status.vel_z = vel_z;
            robot_pos_vel_publisher.publish(robo_status);

            std::cout << "Enter character [ g ] to continue, or [ c ] to cancel: ";
            std::cin >> goal_input;
            if (goal_input == "g") {
                // Get x and y position from user
                std::cout << "Enter required goal position x y: ";
                std::cin >> x >> y;
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;
                target_ac.sendGoal(goal);
                // Setting parameter for goal position for node C
                goal_map["x"] = x;
                goal_map["y"] = y;
                n->setParam("/robot/goal_pos_param", goal_map);
                // break;
            }
            else if (goal_input == "c") {
                std::cout << "Cancelling..." << std::endl;
                target_ac.cancelGoal();
                // Publish number of times the user has cancelled the goal to parmeter server /robot/goal_cancelled
                int goal_cancelled;
                if (n->hasParam("/robot/goal_cancelled")){
                    n->getParam("/robot/goal_cancelled", goal_cancelled);
                    goal_cancelled++;
                    n->setParam("/robot/goal_cancelled", goal_cancelled);
                }
                else {
                    goal_cancelled = 0;
                    n->setParam("/robot/goal_cancelled", goal_cancelled);
                }
                // break;
            }
            else {
                std::cout << "Invalid input. Please enter either 'g' or 'c' again." << std::endl;
            }

            // Check for new messages from the subscribed topics
            ros::spinOnce();

            // Check if the action is finished
            finished_before_timeout = target_ac.waitForResult(rrate->expectedCycleTime());
        }


        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = target_ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
            // Publish number of times the robot has reached the goal to parmeter server /robot/goal_reached
            int goal_reached;
            if (n->hasParam("/robot/goal_reached")){
                n->getParam("/robot/goal_reached", goal_reached);
                goal_reached++;
                n->setParam("/robot/goal_reached", goal_reached);
            }
            else {
                goal_reached = 0;
                n->setParam("/robot/goal_reached", goal_reached);
            }
            
        }
        else
            ROS_INFO("Action did not finish before the time out.");

    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // This function gets the x and y pos, and linear x and y velocities from odom.
        // Position
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
        // Velocity
        vel_x = msg->twist.twist.linear.x;
        vel_z = msg->twist.twist.linear.z;
    }
};


int main (int argc, char **argv) {
    // Initialize the robot_nav_client node
    ros::init(argc, argv, "robot_nav_client");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    RoboClient rc(&n, atof(argv[1]));

    ros::spin();

    //exit
    return 0;
}