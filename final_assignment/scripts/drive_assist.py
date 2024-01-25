#! /usr/bin/env python3

"""

.. module:: drive_assist
  :platform: Unix
  :synopsis: Python module for the drive_assist functionalities of the rover


  This node checks user motion commands sent on /suggestion and, if enabled to do so, corrects them to avoid hitting
  obstacles before publishing them on /cmd_vel.

Subscribes to:

  /suggestion where the node teleop_twist_keyboard_py is having its commands remapped to
 
  /scan where the LIDAR sensor is pubblishing its distance readings

Publishes on:

  /cmd_vel the possibly corrected values of desired linear and angular speeds that the rover is asked to travel at

Service:
  /change_operation this node acts as a server for the change operation service, it reads the requested operation mode as a service request
                    and then sets it's current operating mode accordingly.


"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from final_assignment.srv import ChangeOp, ChangeOpResponse
import math
import os

#Variable used by the service:
current_op=''

#This node publishes on the cmd_vel topic to control the robot:
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed=Twist()


dis_th= 1      #initial value for the distance threshold       
min_th=0.15    #minimal distance for maneuvers
max_speed=1.7  #maximum speed for which dis_th gets adjusted



def block_commands(msg):
    """
    Ignores and does not publish Teleop commands to /cmd_vel.

    This function is called when the rover is in the 't' (target pursuit) mode. It ignores Teleop commands
    and does not publish any commands on /cmd_vel.

    Args:
        msg (Twist): The Twist message containing Teleop commands.

    Returns:
        None
    """
    pass

def manual_drive(msg):
    """
    Republishes the same Teleop commands to /cmd_vel.

    This function is called when the rover is in the 'g' (manual drive) mode. It republishes the same Teleop commands
    from /suggestion to /cmd_vel.

    Args:
        msg (Twist): The Twist message containing Teleop commands.

    Returns:
        None

    Global Variables:
        - speed (Twist): The Twist message to be published on /cmd_vel.
        - pub (rospy.Publisher): The publisher for /cmd_vel.
    """     
    
    speed.linear.x = msg.linear.x
    speed.angular.z = msg.angular.z
    pub.publish(speed)

def assisted_drive(msg):
    """
    Adjusts and publishes linear and angular speeds based on the received Teleop commands.

    This function is called when the rover is in assisted driving mode ('b'). It adjusts the linear speed to the current
    linear speed limits and readjusts the distance threshold (`dis_th`) based on the current linear speed.

    Args:
        msg (Twist): The Twist message containing Teleop commands.

    Returns:
        None

    Global Variables:
        - dis_th (float): The distance threshold for obstacle avoidance.
        - max_speed (float): The maximum linear speed limit.
        - min_th (float): The minimal distance threshold for maneuvers.
        - speed (Twist): The Twist message to be published on /cmd_vel.
    """

    
    global dis_th
    if msg.linear.x>max_speed:
        msg.linear.x=max_speed

    #readjusting dis_th to the current linear speed
    if(msg.linear.x>0):
        dis_th= math.pow( math.log( math.pow(msg.linear.x,3)+1),0.1)-0.6
    
    if dis_th<min_th:
        dis_th=min_th 

    #preparing linear and angular speed, to be published at the next publication on the /scan topic
    speed.linear.x = msg.linear.x
    speed.angular.z = msg.angular.z


def default(msg):

    """
    Prints a message indicating waiting for an operational mode to be set.

    This function is used as a default action when an unrecognized command is received. It prints a message to the
    console indicating that the system is waiting for an operational mode to be set.

    Args:
        msg: Unused parameter.

    Returns:
        None
    """
    print("Waiting for an operational mode to be set", end = "\r", flush=True)

    pass

def clbk_mediate(msg):
    
    """
    Mediates the execution of different actions based on the current operational mode.

    This function is called when commands are sent by the teleop_keyboard node to the /suggestion topic. It uses a
    dictionary (`run_action`) to interpret requests for the service based on the current operational mode.

    Args:
        msg (Twist): The Twist message containing Teleop commands.

    Returns:
        None

    Global Variables:
        - speed (Twist): The Twist message to be published on /cmd_vel.
        - current_op (str): The current operational mode.
        - run_action (dict): A dictionary mapping operational modes to corresponding actions.
    """
    global speed
    
    #dictionary used to interpret requests for the service
    run_action= {
    't' : block_commands,
    'g' : manual_drive,
    'b' : assisted_drive,

    }

    # Calling the appropriate action based on the current operational mode    
    run_action.get(current_op,default)(msg)

def clbk_obstacle_avoidance(msg):
    
    """
    Handles obstacle avoidance based on laser scan readings.

    This function is called every time laser readings are published on /scan. It calculates minimum distances in different
    regions (right, left, and total front) and adjusts the rover's speed accordingly if it is in assisted driving mode.

    Args:
        msg (LaserScan): The LaserScan message containing distance readings.

    Returns:
        None

    Global Variables:
        - speed (Twist): The Twist message to be published on /cmd_vel.
        - current_op (str): The current operational mode.
        - dis_th (float): The distance threshold for obstacle avoidance.
        - min_th (float): The minimal distance threshold for maneuvers.
        - pub (rospy.Publisher): The publisher for /cmd_vel.
    """

    global regions    
    regions= {
    'right':  min(min(msg.ranges[0:143]), 10),
    'left':   min(min(msg.ranges[576:719]), 10),
    'total_front': min(min(msg.ranges[144:575]),10)
    }

    #This part is executed only when assisted driving has been set by the user_interface node
    if current_op == 'b':
        
        #Something is in front of the rover
        if  regions['total_front']< dis_th and speed.linear.x > 0 :            
            speed.linear.x = 0
            speed.angular.z = 0
            print("\nThere's an obstacle within "+str(round(dis_th,2))+"m")            

        #There's a wall blocking a right turn
        elif regions['right'] < min_th and speed.angular.z < 0 :
            speed.linear.x = 0
            speed.angular.z = 0
            print("\nNot enough space to turn right")            

        #There's a wall blocking a left turn    
        elif regions['left'] < min_th and speed.angular.z > 0:
            speed.linear.x = 0
            speed.angular.z = 0
            print("\nNot enough space to turn left")
            
            
        else:
            
            pass
            
        pub.publish(speed)

  
def handle_change_op(req):

    """
    Handles the change_operation service request.

    This function is called when the change_operation service is called from the user_interface node. It updates the
    global variable `current_op` based on the requested option and clears the screen between operating modes.

    Args:
        req (ChangeOpRequest): The request object containing the desired operational mode.

    Returns:
        ChangeOpResponse: The response indicating the success of the operation.

    Global Variables:
        - current_op (str): The current operational mode.
    """

    global current_op   
    current_op =req.option

    #Clearing the screen between operating modes
    os.system('clear') 

    if current_op =='t':
        print("The Rover is pursuing a target, Teleop commands are blocked   ",
         end = "\r", flush=True)
    
    elif current_op == 'g':
        print("Manual drive engaged, use Teleop commands to move the Rover   ",
         end = "\r", flush=True)        
    
    elif current_op == 'b':
        print("Assisted drive engaged, use Teleop commands to drive the Rover",
         end = "\r", flush=True)
    
    return ChangeOpResponse(True)

def main():

    """
    Main entry point for the drive_asist node.

    This function initializes the ROS node, subscribes to the /suggestion and /scan topics, and provides the
    change_operation service. It then enters the ROS spin loop to keep the node running.

    Args:
        None

    Returns:
        None
    """
   
    rospy.init_node('drive_asist') 
    
    rospy.Subscriber('/suggestion', Twist, clbk_mediate)
    rospy.Subscriber('/scan', LaserScan, clbk_obstacle_avoidance )
    rospy.Service('/change_operation', ChangeOp, handle_change_op)

    
    rospy.spin()

if __name__=='__main__':
    main()