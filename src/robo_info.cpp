/** 
 * \file robo_info.cpp
 * \author Aman Arora (aman.arora9848@gmail.com)
 * \brief Robot information node.
 * \version 1.0
 * \date 15/03/2023
 * 
 * \param [in] /robot/goal_pos_param/x_goal The x position of the goal
 * \param [in] /robot/goal_pos_param/y_goal The y position of the goal
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *   ° /robot/robo_stats
 * 
 * Description:
 * 
 * This node recieves the position and velocity of the robot from the topic /robot/robo_stats.
 * It also obtains the user given goal position from the parameter server.
 * The parameter server variables have been configured in the configuration file "robot.yaml".
 * 
 * It thus calculates the average speed of the robot in the x and y directions using these values.
 * 
 */

#include <ros/ros.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>

/**
 * \class RoboInfo
 * 
 * \brief Robot information class.
 * 
 * This class is used to create a node that subscribes to the topic /robot/robo_stats and 
 * obtains the position and velocity of the robot. 
 * It also obtains the user given goal position from the parameter server.
 * 
 * Member functions:
 *  - RoboInfo(ros::NodeHandle *n, double freq)
 *   + This is the constructor of the class.
 *  - robo_info_callback(const unige_rt1_assignment2::RoboStatusMsg::ConstPtr& msg)
 *   + This is the callback function for the subscriber.
 *  - distance_from_goal(double x, double y)
 *   + This function calculates the distance of the robot from the goal position.
 *  - average_speed()
 *   + This function calculates the average speed of the robot in the x and y directions.
 * 
 */
class RoboInfo {
    private:
    ros::Subscriber robot_pos_vel_subscriber; /**< Subscriber to robot position and velocity using custom message type */
    // Goal position, total velocities and array for average speeds
    double x_goal = 0.0, y_goal = 0.0, vx_tot = 0, vy_tot = 0; /**< Goal position, total velocities and array for average speeds */
    std::array<double, 3> average_speeds; /**< Array for average speeds */

    double pos_x, pos_y, vel_x, vel_y; /**< Variables for robot position and velocity */
    int elapsed_time = 0; /**< Variable for elapsed time */

    public:
    RoboInfo(ros::NodeHandle *n, double freq) {
        /**
         * \brief Constructor of the RoboInfo class.
         * \param n Pointer to the node handle.
         * \param freq Frequency of the node.
         * 
         * \return None
         * 
         * This function created a subscriber to the topic /robot/robo_stats to get the robot position and velocity.
         * It also obtains the user given goal position from the parameter server.
         * The Subscriber calls the callback function 'robo_info_callback'.
         * 
         */
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
        /**
         * \brief Callback function for the subscriber.
         * \param msg Pointer to the message.
         * 
         * \return None
         * 
         * This function gets the x and y pos, and linear x and y velocities from RoboSatusMsg message.
         * 
         */
        pos_x = msg->pos_x; /**< Robot x position set from message */
        pos_y = msg->pos_y; /**< Robot y position set from message */
        vel_x = msg->vel_x; /**< Robot x velocity set from message */
        vel_y = msg->vel_y; /**< Robot y velocity set from message */
        // Add velocities to total velocities and increment elapsed time
        vx_tot += std::abs(vel_x); /**< Total x velocity */
        vy_tot += std::abs(vel_y); /**< Total y velocity */
        elapsed_time += 1; 
        show_info();
    }

    void show_info() {
        /**
         * \brief Function to print robot information.
         * 
         * \return None
         * 
         * This function prints the robot position and velocity, goal position, distance from goal and average speed.
         * 
         */
        std::cout << "\nRobot Information: " << "\n==========================================\n" << std::endl;
        ROS_INFO("Robot position (x, y): (%f, %f)", pos_x, pos_y);
        ROS_INFO("Robot velocity (x, y): (%f, %f)", vel_x, vel_y);
        ROS_INFO("Goal position (x, y): (%f, %f)", x_goal, y_goal);
        ROS_INFO("Distance from goal: %f", distance_from_goal(pos_x, pos_y));
        average_speed(); /**< Function call to calculate average speed */
        ROS_INFO("Average speed in x: %f", average_speeds[0]);
        ROS_INFO("Average speed in y: %f", average_speeds[1]);
        ROS_INFO("Average speed overall: %f", average_speeds[2]);
        std::cout << "\n==========================================\n" << std::endl;
    }

    double distance_from_goal(double x, double y){
        /**
         * \brief Function to calculate distance from goal.
         * \param x Robot x position.
         * \param y Robot y position.
         * 
         * \return distance Distance from goal.
         * 
         * This function calculates the distance of the robot from the goal position.
         * 
         */
        double distance = sqrt(pow(x - x_goal, 2) + pow(y - y_goal, 2)); /**< Distance from goal */
        return distance;
    }

    void average_speed(){
        /**
         * \brief Function to calculate average speed.
         * 
         * \return None
         * 
         * This function calculates the average speed of the robot in the x and y directions.
         * 
         */
        double average_speed_x =  vx_tot / elapsed_time; /**< Average speed in x direction */
        double average_speed_y =  vy_tot / elapsed_time; /**< Average speed in y direction */
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
