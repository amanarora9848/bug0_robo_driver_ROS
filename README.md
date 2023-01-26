Research Track 1 - Assignment 2 (UniGe)
=======================================

This is a client package for the package [assignment_2_2022](https://github.com/CarmineD8/assignment_2_2022) that provides an implementation of an action server, that moves a robot in the environment by implementing the bug0 algorithm. This is the 2nd asignment as part of the course: **Research Track 1** at the University of Genoa, Italy.

## Goals and Overview:
Three (or four) principle nodes have been developed as part of this package:
- **Node 1a** (`target_info_client_node`)
    - Implements an action client, allowing the user to set a target position (x, y), or cancel it.
- **Node 1b** (`robot_status_pub_node`)
    - This node subscribes to `/odom` topic and publishes the retrieved position and velocity values using a custom ROS message (`RoboStatusMsg.msg`) to the topic `/robot/robo_stats`.
- **Node 2** (`goal_info_srv_node`)
    - This ROS service node serves the number of goals reached by the robot and cancelled by the user. To get the number of goals, this subscibes to the topic `/reaching_goal_result` and increments each value accordingly, depending on the returned status of the robot reaching goal or not.
- **Node 3** (`robo_info_node`)
    - This node subscribes to the topic `/robot/robo_stats` to obtain robot's position and velocity using our custom message and displays these parameters along with robot's distance from goal and average speed.

In addition to these, in order to show the implementation of a ROS Service-Client, a node: (`disp_goals_node`) has also been created, that calls the Service Server node `goal_info_srv_node` and prints the number of goals reached and cancelled.

## Requirements
The overall project has been well tested using ROS Noetic and requires RViz and Gazebo for simulation and physics.

## Directory Structure [Package: unige_rt1_assignment2]
```
.
|-- CMakeLists.txt
|-- LICENSE
|-- README.md
|-- config
|   `-- robot.yaml
|-- images
|   `-- rosgraph.png
|-- launch
|   `-- complete_sim.launch
|-- msg
|   `-- RoboStatusMsg.msg
|-- package.xml
|-- src
|   |-- disp_goals.cpp
|   |-- goal_info_srv.cpp
|   |-- robo_info.cpp
|   |-- robot_status_pub.cpp
|   `-- target_info_client.cpp
`-- srv
    `-- GoalInfo.srv
```

## Running the full simulation:

#### Update and upgrade packages:

```shell
sudo apt update && sudo apt upgrade
```

#### First, we need to build a catkin workspace for the project. Navigate to preferred directory, then:
```shell
mkdir -p bug0robo_catkin_workspace/src/
cd bug0robo_catkin_workspace/src/
```

#### Initialize the catkin workspace:
```shell
catkin_init_workspace
```

#### Now clone this repo and the action server repo (assuming user has ssh-auth setup):
```shell
git clone git@github.com:amanarora9848/unige_rt1_assignment2.git
git clone git@github.com:CarmineD8/assignment_2_2022.git
```

#### Navigate back to base directory `bug0robo_catkin_workspace`
```shell
cd ..
```

#### Run `catkin_make` to build the package:
```shell
catkin_make
```

#### Source `setup.bash` to set the environment variables used by ROS
```shell
source devel/setup.bash
```

#### Launch the complete simulation. The launch file for this can be found in `/src/unige_rt1_assignment2/launch/complete_sim.launch` relative to the current diretory:
```shell
roslaunch unige_rt1_assignment2 complete_sim.launch
```
Using this command launches the simulation (particularly the node-C at the default rate of 5 /sec). This can be changed by also passing an argument parameter, for example: 

```roslaunch unige_rt1_assignment2 complete_sim.launch freq:=3```

***Below is an example of the simulation running:
[Implementation Video](https://youtu.be/TDBa1oudBaU)***

## Project Explanation and Implementation

### ROS rqt-graph (with nodes and topics) for the whole project (client-server)

[rqt-graph](images/rosgraph.png)

### ROS Nodes in detail:

This package uses 4 + 1 nodes for the task to be done effectively and holistically. They are `target_info_client_node`, `robot_status_pub_node`, `goal_info_srv_node`, `robo_info_node` and 1 extra node: `disp_goals_node`. 

- As seen from the rqt-graph, the action client node `target_info_client_node` interacts with the action server defined in the package `assignment_2_2022` using the `Planning.action` file, in which are defined the action goal and feedback messages.

    - The pseudocode for this node is given below:
    ```
    while ros is running:
        take user input to proceed or cancel goal

        if input is proceed:
            if cancel command passed before (is in stack):
                pop cancel command from stack
            end
            take user input for x and y
            send goal to action server
            set goal in parameter server variables
        else if input is cancel:
            if cancel not passed before: // not in stack
                push cancel command to stack
                if action server status is executing goal:
                    send cancel to action server
                else:
                    prohibit cancel command
                end
            else if cancel in stack:
                prohibit cancel command
            end
        else:
            prohibit cancel command
        end

    ```

- The `robot_status_pub_node` subscribes to the `/odom` topic for position and velocity of the robot and publishes them at the topic `/robot/robo_stats`.
    - The pseudocode for this node is given below:
    ```
    subscribe to /odom topic
    check for /odom callbacks
    set x, y, vel_x, vel_y in callbacks
    while ros is running:
        set user x, y, vel_x, vel_y obtained from /odom
        publish these as custom message to /robot/robo_stats topic at user defined rate

    ```
    - Given below is the custom message definition.
    ```
    # A custom message for robot position and velocity
    float32 pos_x
    float32 pos_y
    float32 vel_x
    float32 vel_y
    ```

- The node `robo_info_node` recieves these robot position and velocity values by subscribing to the `/robot/robo_stats`, along with obtaining the user-given goal position from the parameter server. The parameter server variables have been configured in the configuration file `robot.yaml` present in the ***config*** folder.
    ```yaml
    robot: {
        goal_pos_param: {
            x_goal: 0,
            y_goal: 0
        }
    }
    ```
    - Using these values, the distance to the goal and theaverage velocities are calculated.

- The node `goal_info_srv_node` also interacts with the action server messages and receives the status message (`assignment_2_2022::PlanningActionResult::ConstPtr& msg`) from it, to determine the number of goals reached or cancelled.
    - The pseudocode for this node:
    ```
    Declare service goal_service

    Subscribe to /reaching_goal/result topic from action server
    Check for /reaching_goal/result callbacks

    if goal is completed:
        goals_reached_number = goals_reached_number + 1
    else if goal is cancelled:
        goals_cancelled_number = goals_cancelled_number + 1
    end

    Define service response message
    Advertise /goals_service service
    ```
    - Given below is the service (srv) file definition.
    ```
    ---
    string msg_feedback
    ```

- The extra node `disp_goals_node` simple sends requests to the service server node `goal_info_srv_node` at the specified rate defined in the launch file (which can be set by the user, the same for the node `robo_info_node`) to get the number of goals reached or cancelled and prints them on the terminal. The simple class definition for the node is given below:
    ```cpp
    class GoalInfoClient {
        private:
        // Define the service client
        ros::ServiceClient goal_info_client;
        // Define the service message
        unige_rt1_assignment2::GoalInfo goal_info_srv;

        public:
        GoalInfoClient(ros::NodeHandle *n, double freq) {
            // Set the frequency of the loop
            ros::Rate *loop_rate;
            loop_rate = new ros::Rate(freq);

            // Create the service client
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
    ```


## Programming style

This package uses C++ classes for implementation of the ROS Nodes, since the use of classes not only provides structure and clarity to the code, but also aids in proper management of data and methods. We also avoid using unnecessary global variables in the code. The programs simply become more modular, with reusable chunks which are beneficial in larger projects.


## Possible Implovements
- The client console can be made even more robust, error-free and interactive, since some of the erroneous user-inputs have not been handled.
- An interactive GUI on the console can be created.
- Currently, there is no exit button / command, and the easiest way to exit simulation is using `Ctrl-C` on the main console, where `roslaunch` command is used. Better mechanism can be developed.


## Helpful references / documentation / answers:

- http://wiki.ros.org/actionlib/DetailedDescription
- http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatusArray.html