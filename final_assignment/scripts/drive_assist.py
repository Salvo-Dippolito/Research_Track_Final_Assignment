#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from final_assignment.srv import ChangeOp, ChangeOpResponse
import math
import os

#Variable used by the srvice:
current_op=''

#This node publishes on the cmd_vel topic to control the robot:
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed=Twist()


dis_th= 1      #initial value for the distance threshold       
min_th=0.15    #minimal distance for maneuvers
max_speed=1.7  #maximum speed for which dis_th gets adjusted



def block_commands(msg):
    
    #not pubblishing any commands on /cmd_vel
    pass

def manual_drive(msg):     
    
    #simply republish the same Teleop commands from /suggestion to /cmd_vel
    speed.linear.x = msg.linear.x
    speed.angular.z = msg.angular.z
    pub.publish(speed)

def assisted_drive(msg):

    
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
    print("Waiting for an operational mode to be set", end = "\r", flush=True)

    pass

def clbk_mediate(msg):
    
    #This is called when commands are sent by the teleop_keyboard node to the /suggestion topic

    global speed
    
    #dictionary used to interpret requests for the service
    run_action= {
    't' : block_commands,
    'g' : manual_drive,
    'b' : assisted_drive,

    }
    
    run_action.get(current_op,default)(msg)

def clbk_obstacle_avoidance(msg):
    
    #Called every time laser readings are pubblished on /scan

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

    #This handle is called when the change_operation service is called from the user_interface node

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
   
    rospy.init_node('drive_asist') 
    
    rospy.Subscriber('/suggestion', Twist, clbk_mediate)
    rospy.Subscriber('/scan', LaserScan, clbk_obstacle_avoidance )
    rospy.Service('/change_operation', ChangeOp, handle_change_op)

    
    rospy.spin()

if __name__=='__main__':
    main()