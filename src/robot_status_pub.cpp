#include <ros/ros.h>
#include <unige_rt1_assignment2/RoboStatusMsg.h>
#include <nav_msgs/Odometry.h>

class RobotStatusPublisher {
    private:
    // Define variables for position and velocity to be published
    float pos_x, pos_y, vel_x, vel_y;
    // ROS::Subscriber listening to odom for robot position and velocity;
    ros::Subscriber robot_pos_vel_subscriber;
    // ROS::Publisher for robot position and velocity;
    ros::Publisher robot_pos_vel_publisher;

    public:
    RobotStatusPublisher(ros::NodeHandle *n, double freq) {
        // Subscribe to /odom topic to read the position and velocity of the robot with the subscribing queue size of 10
        robot_pos_vel_subscriber = n->subscribe("/odom", 1, &RobotStatusPublisher::odom_callback, this);
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        robot_pos_vel_publisher = n->advertise<unige_rt1_assignment2::RoboStatusMsg>("/robot/robo_stats", 10);
        // Publish the robot position and velocity on the robot_pos_vel_publisher Publisher
        ros::Rate *loop_rate;
        loop_rate = new ros::Rate(freq);

        while(ros::ok()) {
            unige_rt1_assignment2::RoboStatusMsg robo_status;
            robo_status.pos_x = pos_x;
            robo_status.pos_y = pos_y;
            robo_status.vel_x = vel_x;
            robo_status.vel_y = vel_y;
            robot_pos_vel_publisher.publish(robo_status);
            ROS_INFO("Robot position: (%f, %f), velocity: (%f, %f) published.", pos_x, pos_y, vel_x, vel_y);
            ros::spinOnce();
            loop_rate->sleep();
        }

        delete loop_rate;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // This function gets the x and y pos, and linear x and y velocities from odom.
        // Position
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
        // Velocity
        vel_x = msg->twist.twist.linear.x;
        vel_y = msg->twist.twist.linear.y;
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "robot_status_pub_node");
    ros::NodeHandle n;
    // Create an instance of the RobotStatusPublisher class
    RobotStatusPublisher robot_status_publisher(&n, atof(argv[1]));
    return 0;
}