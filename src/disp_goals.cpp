/**
 * \file disp_goals.cpp
 * \author Aman Arora (aman.arora9848@gmail.com)
 * \brief Node to display number of goals reached or cancelled.
 * \version 1.0
 * \date 15/03/2023
 * 
 * \details
 * 
 * Services: <BR>
 *  ° /goals_service
 * 
 * Description:
 * 
 * This node is used to create a service client node that sends a request to the service server 
 * /goals_service defined in this package, and, as response, displays the number of goals reached
 * or number of goals cancelled by the user, by pressing 'q' on the user CL-interface.
 * 
 */

#include <ros/ros.h>
#include <unige_rt1_assignment2/GoalInfo.h>

/**
 * \class GoalInfoClient
 * \brief Classd defining the service client
 * 
 * This class defines the service client that sends a request to the service server /goals_service
 * displays the number of goals reached or cancelled by the user.
 * 
 */
class GoalInfoClient {
    private:
    ros::ServiceClient goal_info_client; /**< Service client to send request to the service server */
    unige_rt1_assignment2::GoalInfo goal_info_srv; /**< Service message definition to send request to the service server */

    public:
    GoalInfoClient(ros::NodeHandle *n, double freq) {
        /**
         * \brief Constructor of the class
         * 
         * This constructor initializes the service client and the service message definition.
         * 
         * \param n Pointer to the node handle
         * \param freq Frequency of the loop
         * 
         * \return None
         * 
         * This constructor initializes the service client and the service message definition.
         * 
         */
        ros::Rate *loop_rate; /**< variable to set the frequency of the loop */
        loop_rate = new ros::Rate(freq); /**< Initialize the loop rate variable */

        /**
         * \brief Initialize the service client
         * 
         * This statement initializes the service client to send request to the service /goals_service server.
         * 
         */
        goal_info_client = n->serviceClient<unige_rt1_assignment2::GoalInfo>("/goals_service");
        
        while(ros::ok()) {
            // Call the service
            if (goal_info_client.call(goal_info_srv)) {
                ROS_INFO_STREAM(goal_info_srv.response.msg_feedback);
            }
            else {
                ROS_ERROR("Failed to call service /goals_service");
            }
            // check for incoming messages
            ros::spinOnce();
            // sleep for the time remaining
            loop_rate->sleep();
        }

        delete loop_rate;
    }
};

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "disp_goals_node");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Create the goal info client object
    GoalInfoClient goal_info_client(&n, atof(argv[1]));
    
    // exit
    return 0;
}