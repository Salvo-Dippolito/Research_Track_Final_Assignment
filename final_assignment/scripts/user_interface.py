#! /usr/bin/env python3

from http import client
import rospy
import sys, select, termios, tty
import os


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from final_assignment.srv import ChangeOp, ChangeOpResponse


change_op=rospy.ServiceProxy('/change_operation',ChangeOp)

options_list="""

        === HOME ROVER CONTROL OPTIONS ===
----------------------------------------------------
  1) Fix a new target position: press 't'

  2) Manual control: press 'g'

  3) Manual control with collision assist: press 'b'

  To quit simulation: press 'q'

"""

def get_option():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], None)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def not_a_float(value):
    try:
        float(value)
    except:
        return 1
    else:
        return 0

def get_coordinates():
    x=''
    y=''

   
    while not_a_float(x) or not_a_float(y):

        x= input('Enter the target x coordinate: ')
        y= input('Enter the target y coordinate: ')
    
    return float(x), float(y)

class Set_goal():

    #Class to send and cancel goals to the move_base action client
    def __init__(self):
        self.reached = 0
        self.aborted = 0
        self.rejected = 0
        self.flag = 0
        
        print("    --Goal Oriented Navigation--                               \n")

        #Get x and y coordinates from user:
        goal_x, goal_y= get_coordinates()

        #Setting up as a client:
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
    
        #try_goal sends the desired goal pose and handles exceptions
        self.try_goal( goal_x, goal_y)

        #waiting for a result from the action server:
        self.client.wait_for_result()

        #Clearing and resetting the screen:
        os.system('clear')
        print(options_list)


    def done_cb(self, status, result):

    # This callback interprets the received goal status for the user 
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            print("  Received a cancel request\n\n\r  Press any key to go back to options\n\r")
            
        if status == 3:
            print("  Goal reached!\n\n\r  Press any key to go back to options\n\r")
            self.reached = 1

        if status == 4:
            print("  Can't reach desired target\n\n\r  Goal aborted by action server\n\r  Press any key to go back to options\n\r")
            self.aborted = 1            

        if status == 5:
            print("  Goal rejected by action server\n\n\r  Press any key to go back to options\n\r")
            self.rejected = 1          

    def active_cb(self):
        print("  To cancel current goal: press 'c'\n\r", flush=True)

    def feedback_cb(self, feedback ):
        
        #Uncomment this to have the feedback printed on the terminal: 
        #print(str(feedback))
        pass
    
    def something_happened(self):
        if self.reached == 1 or self.aborted == 1 or self.rejected == 1 :
            
            self.reached = 0
            self.aborted = 0
            self.rejected = 0
            self.flag=1

            return 1

    def try_goal(self,goal_x,goal_y):

        goal=MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y   
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        print("target goal sent, wait for the robot to reach the goal or cancel it\n \r", flush=True)
        
        while(get_option()!='c'):
            
            if self.something_happened()==1:
            #If something happens before command c is received, stop waiting for such command
                break
        
        #If the loop was exited because something hapened:
        if self.flag == 1:
            self.flag = 0
            #Just reset the flag

        #If the loop was exited because a cancel command was received:
        else:
            self.client.cancel_goal()
            get_option()
     
def set_manual_drive():
    print(" Manual drive engaged, to control the robot use the Teleop terminal  ", end = "\r", flush=True)

def set_assisted_drive():
    print(" Assisted drive engaged, to control the robot use the Teleop terminal", end = "\r", flush=True)

def close():

    #This shuts down the ROS system when called
    
    print("Sending a shutdown signal                                            ", end = "\r", flush=True)
    rospy.signal_shutdown("requested shutdown")

def default():
    print(" Invalid option, select fom the menu", end = "\r", flush=True)
 
def change_op_client(operation):

    #This takes in the desired user command and sends it as a request to the change_operation service
    
    global current_op
    rospy.wait_for_service('/change_operation')    

    try:
        change_op(operation)       

    except rospy.ServiceException as e:
        print("Couldn't execute service call: %s" %e) 

def main():

    run_action= {
    't' : Set_goal,
    'g' : set_manual_drive,
    'b' : set_assisted_drive,
    'q' : close,
    }

    #For the get_option function:
    global settings
    settings = termios.tcgetattr(sys.stdin)
   
    rospy.init_node('user_interface')

    
    print(options_list)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        #Take a command from user:
        option=get_option()

        #Calling the change_operation service to set the current operation mode:
        change_op_client(option)

        #Running the desired function requested from the user:
        run_action.get(option,default)()
        
        rate.sleep()

if __name__ == '__main__':
    main()
