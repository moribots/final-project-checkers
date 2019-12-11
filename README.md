# Baxter Plays Checkers!
## Maurice Rahme, Robert Schloen, Jordan Zeeb, Taoran Zhang
****
## Project Overview
This package allows a Baxter robot to play checkers using the modern ruleset! You have a choice to play using our custom checkers AI, or against a human-operated Baxter robot. This project was completing as part of the final project requirement for ME495: Embedded Systems in Robotics taught by Prof. Matthew Elwin. This project placed 1st in the judged competition between a total of 6 project teams. The theme for this year's competition was recreational robotics, and the other teams' projects were: Terminator (a nerf-gun shooting robot), Mini-Golf, Cornhole, Tic-Tac-Toe and MegaBLOKS. If you liked this project, you should check them out!

Here is a video of an AI-operated game:

**ADD VIDEO**

Here is a video of a human-operated game:

**ADD VIDEO**

## Quickstart guide
* Download `checkers.rosinstall` to your `/src` directory in your catkin workspace
* `wstool merge` in the same directory. This will download the checkers package and my modified moveit_robots package.
* Make sure you have the rethink robotics workspace installed and sourced.
* Run catkin_make in your catkin workspace after sourcing it.
* Set up the game table.
* Plug Baxter's ethernet cable into your computer.
* Connect to baxter using ` ROS_MASTER_URI=http://10.42.0.2:11311` and `export ROS_IP=10.42.0.1`.
* `roslaunch roslaunch checkers moveit_motion_plan.launch`
* In a separate terminal, `rosrun checkers pick_place_as`
* In a separate terminal, `rosrun checkers smach` (this is the main terminal you will interface with, the others are for debugging and diagnostics)
* Follow the instructions on the terminal where you opened `smach`, you are ready to play!


## Contribution guide (if you are a team member)

**SETUP**

* Fork this repository.
* Use `wstool set` with the SSH to YOUR repository, NOT the group one to automatically set the origin master.
* Then, do `git remote add upstream git@github.com:ME495-EmbeddedSystems/final-project-checkers.git`.
* Now you can push to upstream using `git push upstream` or your own repo using `git push origin` or both using `git push --all`.

**WORKFLOW**

Work on your own branch and push to `origin` whenever you want to back up your progress.

When you are ready to push to `upstream` (the shared repo):

* `git checkout master`
* `git pull upstream master`.
* `git checkout <your_personal_branch>`
* `git merge master`
* Make sure code works and fix any bugs.
* Once everything works, `git checkout master`, and `git merge <your_personal_branch>`
* `git push upstream` and `git push origin`

****
## Nodes and Launchfile

### moveit_motion_plan.launch
This launchfile begins by launching the `joint_trajectory_action_server` from the `baxter_interface` package, which `MoveIT!` uses to actuate baxter's arms after motion planning. The `baxter_grippers.launch` file from the modified `baxter_moveit_config` package is also included, which loads various moveit parameters pertaining to this Baxter, such as the number of joints in each arm, the types of grippers, and the Inverse Kinematics Solver (Here, we are using TRAC-IK with the Distance Solve Type).

Next, it launches the `bridge` node, which is the computer vision node that reads the board state and sends it to `smach`. It also ensures that both the left and right hand cameras are turned on with maximum resolution (1280x800). Finally, it launches the `screen` node, which displays either the processed image from `bridge`to Baxter's screen, or the image from the pick-place end-effector, depending on instructions by `smach`.

### smach
This node is the main interface between the other nodes and scripts in the checkers package. It serves as a state machine with 8 states as shown by the diagram below.

![State Machine](media/smach.svg)

**SETUP**: The user is asked to place Baxter's left end-effector on the top-left crosshairs of the bottom-right square of the board relative to baxter using the provided calibration grippers. Once the position is set, the user can hit Enter on the terminal, and Baxter will move its right end-effector to the location it believes to be the centre of the top-left square of the board. If this belief is false, adjust the board/table and repeat the calibration as necessary. Once the user is satisfied with the calibration, they can hit Enter again to move on to the next phase. 

**PICKCOLOR**: By default, the user is asked to select a color, purple or green, for Baxter. Alternatively, using the `bridge` note and the `play_checkers.py` script, Baxter can figure out its own color by identifying that of the piece closest to it. If Baxter is purple, the user goes first, and vice versa. Once the user hits enter, the state machine will move either to the **WAIT** or **MYTURN** phase depending on the color choice. 

**WAIT**: Here, Baxter waits for the user to play their move. Once the user is done, they can hit Enter to move to the **MYTURN** state.

**MYTURN**: An action client set up within `smach` sends a joint goal to the `pick_place_as` action server to move the right (pick-place) arm to its home position, away from the board, and subsequently the left (camera) arm above the board. Once this is done, the `bridge` node reads the board state and sends a string containing the status of each square (empty, green, or purple) in row-major-order to the `Board_State` topic, which `smach` subcribes to and sends to the `CheckersAI` class either using the `minimax` method, which generates a move using the minimax algorithm sped-up with alpha-beta pruning, or using the `give_command` method, which displays the list of legal moves to the user and asks them to input their choice in the terminal. If the board state indicates that Baxter has lost, the **SHUTDOWN** state is enqueued. Otherwise, the move lit is returned, a list containing at least two integers between `0-63` indicating the board squares in row-major order; these are the pick and place square locations for baxter's move. List elements in addition to the first two indicate squares where pieces must be discared as they have been captured by Baxter's move. Once the move list is generated, it is sent as `userdata` through `smach` into the **MOVE** state. The camera arm is also returned to its home position by sending a joint goal to the action server to avoid obstructing the board. 

Alternatively, the user can choose to decline the AI's suggested move, or to play a move themselves for whatever reason. In this case, a single-element move list containing `None` is passed. In addition, the processed image from the `bridge` node is shown here by sending "left" to the `arm_img` topic, which the `screen` node subscribes to.

**MOVE**: The moves list is read as `userdata` through `smach`. If the first element contains `None`, a `Warning` is printed saying that no move was played, and the **WAIT** state is enqueued. Otherwise, the action server is sent a pick and place goal using the first two elements of the moves list respectively, and discard goals for every additional element in the list. The cartesian (x,y) coordinates are extracted from the moves list using a stored dictionary which multiplies the width of a square by the number of squares in every direction, and adds this to an offset calclated during the calibration method in the **SETUP** state. The discard place location is hard-coded. These goals are executed using `MoveIt!`'s Python wrapper for their cartesian path planner. While this is happening, the video feed from the right arm camera is shown by sending "right" to the `arm_img` topic, which the `screen` node subscribes to.

**SHUTDOWN**: Here, the user is asked to put Baxter's arms in a safe position using a `Warning` type message. Then, once the user hits enter, Baxter is disabled.


### pickplace_as
This node starts an action server which receives PickPlace.action goals of various types and actuates Baxter's arms to perform them using `MoveIt!`'s `moveit_commander` Python wrapper. It can perform the `pick_cartesian` and `place_cartesian` methods, the `joint_goal` method, and the `pose_goal` method, although the latter is only used in case the first two fail to find an IK solution. The different goal types are:

* **home**: if `True`, sets the right arm to its predefined home configuration. 
* **camera_home**: if `True`, sets the left arm to its predefined home configuration.
* **camera_view**: if `True`, sets the left arm to its predefined board view configuration.
* **calibrate**: if `True`, records the left end-effector's cartesian coordinates and saves them to be added as an offset to any given `pick` or `place` goals in the future.
* **pick_goal**: Two element float list, containing the x,y position of the `pick` goal to be executed by Baxter's right arm.
* **place_goal**: Two element float list, containing the x,y position of the `place` goal to be executed by Baxter's right arm.

The first three goals are actuatied using the `joint_goal` method, while the latter two are actuated using the `pick_cartesian` and `place_cartesian` methods which call `MoveIt!`'s cartesian path planner.

### play_checkers.py

### bridge

### screen

****
## System Architecture

## Future Work
A big improvement would be to add a re-planning feature using Force Control. For example, if Baxter detects that it should have picked up a piece when it in fact did not, it should try the move again. This should be easy to achieve using the existing structure of the action server. In addition, while the `MoveIt!` methods were generally robust (an average of 3 failures per 30-minute game), some further optimisation could be done, such as having more hard-coded home positions for the picking arm depending on reachability, using the camera arm as a secondary picking arm, or performing the motion planning from scratch by using the methods described in [Modern Robotics Package](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python). In addition, `MoveIt!` seems more suited for use with C++ as opposed to Python, as it contains more methods and more robust ones for those also supported in Python.

In addition, calibrating the z-coordinate for the pick-place actions is tedious, and Force control could be used to semi-automate that process for differently sized or situated tables.