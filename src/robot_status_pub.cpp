/**
 * \file robot_status_pub.cpp
 * \author Aman Arora (aman.arora9848@gmail.com)
 * \brief Status publisher node for the robot.
 * \version 0.1
 * \date 2023-03-15
 * 
 * \details
 * 
 * Subscribes to: <BR>
 *   ° /odom
 * Publishes to: <BR>
 *  ° /robot/robo_stats
 * 
 * Description:
 * 
 * This node is used to publish the position and velocity of the robot on the topic /robot/robo_stats.
 * 
 * It uses the message RoboStatusMsg ROS message definition defined in this package.
 * 
 */

#include <ros/ros.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>
#include <nav_msgs/Odometry.h>


/**
 * \class RobotStatusPublisher
 * \brief Publishes the position and velocity of the robot.
 * 
 * This class is used to publish the position and velocity of the robot on the topic /robot/robo_stats.
 * 
 */
class RobotStatusPublisher {
    private:
    float pos_x, pos_y, vel_x, vel_y; /**< Position and velocity of the robot */
    ros::Subscriber robot_pos_vel_subscriber; /**< Subscriber object to the /odom topic */
    ros::Publisher robot_pos_vel_publisher; /**< Publisher object to the /robot/robo_stats topic */

    public:
    RobotStatusPublisher(ros::NodeHandle *n, double freq) {
        /** Subscribe to /odom topic to read the position and velocity of the robot with the subscribing queue size of 1 and call the odom_callback function */
        robot_pos_vel_subscriber = n->subscribe("/odom", 1, &RobotStatusPublisher::odom_callback, this);
        /** Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10 */
        robot_pos_vel_publisher = n->advertise<unige_rt1_assignment2::RoboStatusMsg>("/robot/robo_stats", 10);
        /** Publish the robot position and velocity on the robot_pos_vel_publisher Publisher object at the rate of freq Hz */
        ros::Rate *loop_rate; /*< Loop rate object */
        loop_rate = new ros::Rate(freq);

        while(ros::ok()) {
            unige_rt1_assignment2::RoboStatusMsg robo_status; /*< Robot status message object */
            robo_status.pos_x = pos_x; /**< Robot position x */
            robo_status.pos_y = pos_y; /**< Robot position y */
            robo_status.vel_x = vel_x; /**< Robot velocity x */
            robo_status.vel_y = vel_y; /**< Robot velocity y */
            robot_pos_vel_publisher.publish(robo_status); /*< Publish the robot status message */
            ROS_INFO("Robot position: (%f, %f), velocity: (%f, %f) published.", pos_x, pos_y, vel_x, vel_y);
            ros::spinOnce();
            // Sleep for the time remaining to let us hit our publish rate
            loop_rate->sleep();
        }

        delete loop_rate;
    }

    // This function gets called whenever a new message arrives on the /odom topic
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        /**
         * \brief Callback function for the /odom topic.
         * \param msg Message received on the /odom topic.
         * 
         * \return void
         * 
         * This function gets called whenever a new message arrives on the /odom topic.
         * It reads the position and velocity of the robot from the message and updates 
         * the robot position and velocity.
         * 
         */
        pos_x = msg->pose.pose.position.x; /**< Robot position x */
        pos_y = msg->pose.pose.position.y; /**< Robot position y */
        vel_x = msg->twist.twist.linear.x; /**< Robot velocity x */
        vel_y = msg->twist.twist.linear.y; /**< Robot velocity y */
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "robot_status_pub_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Create an instance of the RobotStatusPublisher class
    RobotStatusPublisher robot_status_publisher(&n, atof(argv[1]));

    // exit
    return 0;
}