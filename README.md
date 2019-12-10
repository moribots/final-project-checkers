# Baxter Plays Checkers!
## Maurice Rahme, Robert Schloen, Jordan Zeeb, Taoran Zhang
****
## Project Overview

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

* `pull` master branch from `upstream`.
* `merge` master branch into your personal branch.
* Make sure code works and fix any bugs.
* Once everything works, merge your personal branch back into master.
* `push` to `upstream` (and to origin as your personal backup).

## System Architecture