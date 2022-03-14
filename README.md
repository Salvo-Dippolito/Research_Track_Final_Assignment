# Research_Track_Final_Assignment

### Exercise Objectives ###

The robot should be able to operate in three different modes:

  - Point to point navigation to targets given by the user.  
  - Manual driving.  
  - Manual driving with automated obstacle avoidance.

The first mode must be carried out by comunicating to the move_base action server. Manual control of the robot should 
be carried out either by building a new control node or by remapping the teleop_keyboard_twist node onto a new topic 
that publishes for a node that in turn mediates between teleop commands and user defined operating modes.


### Chosen Approach to Complete the Objectives ###

This solution uses the latter approach, consisting in the development of a user_inteface node and a drive_assist node 
that handles teleop commands according to what is asked by the user interface.
Goal requests on the other hand, are sent directly by the user_inteface node. The node announces itself as an action 
client, thus utilizing the feedback and status funcionalities offered by the action client architecture.

The drive_assist node subscribes to two topics: the /suggestion topic, to which the teleop node is remapped to by the 
launch file, and the /scan topic, where all the laser sensor readings are periodically pubblished. It also fixes 
itself as a publisher to the /cmd_vel topic, where the desired poses are sent to the model of the robot. Under manual 
drive, this node simply republishes the same commands it received from the teleop node onto the /cmd_vel topic. 
When assisted drive is engaged, all drive commands get cross checked with the received sensor readings to anticipate 
possible collisions: if the robot is driving towards a wall, the node will stop it as soon as it comes under a 
certain distance of such wall. This distance threshold is calculated from the current speed of the robot every time a 
new command is received on the /suggestion topic. The formula used is supposed to increase this threshold for higher 
speeds and to drastically reduce it for lower speeds. Finally, when the robot is pursuing a target set by the user 
interface node, the drive assist node stops pubblishing commands on the /cmd_vel topic, effectively shutting down 
teleop controls.

The drive_assist node is ordered which operational mode it should execute through a service call sent by the 
user_interface node.  

### Pseudo-code for the Solution that has been Implemented ###
#### user_interface.py ####
    
    initialize node
    
    while rospy isn't shut down:
      
      get a keyboard input
      
      call change_operation service to change drive_assist operation mode
      
      run the part of code corresponding to the desired option:
              't': Set_goal():
                  ask user for 2 coordinates
                  
                  setup as an action client
                  
                  send coordinates as goal
                      if 'c' is pressed and no change of status has occoured cancel goal
                      
                      else: notify change of status
                  
                  wait for any key to be pressed
                  reset screen terminal
             
              'g': manual_drive():
                  notify the user about the current mode and to use telep terminal

              'b': assisted_drive():
                  notify the user about the current mode and to use the teleop terminal
                  
      wait 1/20th of a second
     
#### drive_assist.py ####
    globally declare a handle to pubblish on /cmd_vel
    create appropriate message object to pubblish on /cmd_vel
    
    initialize some driving variables
    
    initialize node
    subscribe to /suggestion
    subscribe to /scan
    establish /change_operation service
    
    execute callbacks when something is published or the service is called:
    
        - handle_change_op(when a service call is received with user set option):
            
            save request in the global variable current_op
            notify the user that a change in operation has been issued
        
        
        - clbk_mediate(when a command is pubblished on /suggestion):
        
        
            run the part of code corresponding to the value of current_op:
              't': block_commands():
                  do nothing
               
             
              'g': manual_drive():
                  read linear.x and angular.z from the Twist message pubblished on /suggestion
                  republish them on /cmd_vel

              'b': assisted_drive():
                  check current speed
                  if the robot is moving forward recalculate distance threshold to match current speed
                  
                  if the calculated distance threshold is too low assign a preset minimal value
                  
                  prepare linear and angular speeds to be checked by the laser callback
            
        
        - clbk_obstacle_avoidance( when the laser sensor pubblishes on /scan):
        
        
            split all readings in three critical regions and pick the smallest for each region
            
            if current_op has been set to 'b' by the service:
                
                if moving forward and wall in front:
                  stop motors and notify user
                  
                if turning right and no space to move:
                  stop motors and notify user
                
                if turning left and no space to move:
                  stop motors and notify user
                
                if none of the above
                  do nothing, not changing speeds set by mediate calback in assisted_drive mode
                
                pubblish the spped values that have been set
 
### Contents of the Package ###

- launch folder: contains the main launch file home_rover.launch and the other two launch files called by 
                 the main one

- srv folder: for the ChangeOp.srv file used by the service

- scripts folder: contains the two python nodes developed
 
- worlds folder: contains the 3d environment used in this assignment
 
- urdf folder: info about the rover composition for the gazebo simulation
 
- config folder: needed for Rviz 
 
- param folder: parameters needed for the slam_gmapping and move_base nodes 

- CmakeLists.txt and package.xml files are for ros functionalities

### Launching the Simulation ###

1) Clone this repository in your ros workspace folder
2) Make sure you have xterm installed, it is used by the launch file to open the three terminals.
3) This package depends on two packages called gmapping and slam_gmapping which are included in this repository
   Make sure to also clone these in your ROS workspace

4) Check also if the ROS navigation stack and the teleop_twist_keyboard package are installed:

```
$ sudo apt-get install ros-<your_ros_distribution>-navigation
```
and
```
$ sudo apt-get install ros-<your_ros_distribution>-teleop-twist-keyboard
```
5)Once all packages have been installed, run this from the workspace directory:

```
$ catkin_make
```
6) To finally launch the simulation run this command in any terminal:

```
$ roslaunch final_assignment home_rover.launch
```
To actually control the robot just follow the instructions printed on the user_interface terminal.
### Future Improvements ###

There are some issues with launching the Gazebo and Rviz environments with this ros noetic version, everything runs 
but the main terminal gets spammed with warnings. This might conceal one of the reasons of why it can take up to 
two or three minutes for the rover to receive the correct status update after it has reached its user-defined goal. 
One great improvement for ease of use would be to find a way to have all three terminals bundled in one macro 
terminal at launch, so the user wouldn't have to move three different terminals at the same time. Finally the 
change_op_client() function should be moved at the start of the set_goal set_manual and set_assisted functions so 
that the server gets called only for valid keyboard inputs, leaving everything as it is however doesn't change the 
user experience in any significant way.
                
                
                
                
            
    
          
    
    

    

    
