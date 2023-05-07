#!/usr/bin/env python3
import rospy
import actionlib
from assignment_2_2022.msg import PlanningAction, PlanningGoal
from std_msgs.msg import Float64
import sys


class RoboClient:
    def __init__(self):
        # create the action client, True causes the client to spin its own thread
        self.target_ac = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)

        rospy.loginfo("Waiting for action server to start.")
        self.target_ac.wait_for_server()  # Wait for the action server to start, will wait for infinite time

        rospy.loginfo("Action server started! Press [ ENTER ] to request a goal.")

        print("\n==========================================\n")

        self.goal_input = ""
        self.goal_stack = []

        while not rospy.is_shutdown():
            self.goal_input = input("Enter [ p ] to input required goal position, or enter [ q ] to cancel the current set goal: ")

            # Check if user input is p or q
            if self.goal_input == "p":
                # Check if q was previously entered and remove it from the stack
                if self.goal_stack and self.goal_stack[-1] == "q":
                    self.goal_stack.pop()

                x, y = map(float, input("Enter required goal position x y: ").split())

                goal = PlanningGoal()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                self.target_ac.send_goal(goal)

                # Set the goal positions in the parameter server variables
                if rospy.has_param("/robot/goal_pos_param"):
                    rospy.set_param("/robot/goal_pos_param/x_goal", x)
                    rospy.set_param("/robot/goal_pos_param/y_goal", y)
                else:
                    rospy.logerr("Goal position parameter not found on the parameter server.")

            elif self.goal_input == "q":
                # Ensure that q has not already been pressed by user
                if not self.goal_stack or self.goal_stack[-1] != "q":
                    self.goal_stack.append("q")

                    # Check if there is a goal currently executing
                    if self.target_ac.get_state() == actionlib.SimpleClientGoalState.ACTIVE:
                        print("Cancelling current goal...")
                        self.target_ac.cancel_goal()
                    else:
                        print("No goal is currently executing.")
                else:
                    print("Invalid input. Please press [ ENTER ] to input x and y values.")
            else:
                print("Invalid input. Please enter [ ENTER ] or 'q' again.")

            print("\n==========================================\n")

if __name__ == '__main__':
    # Initialize the robot_nav_client node
    rospy.init_node('target_info_client')

    # Create the robot action client object
    rc = RoboClient()

    # Keep the node running until it is stopped externally\
    rospy.spin()

    # exit
    sys.exit(0)