Keegan Nave - ROB599 - HW1


LAUNCH FILE:
    Run everything: roslaunch rob599_hw1 wall_stare.launch 
        - runs approach, filter, and wall angle node


TESTING:
    After running the launch file the Fetch robot will go 1 meter to the wall in front of it unless a service call or action server call is made.
    Robot will get within +-0.07 meters of any given location

    Topics/Markers:
        - Filtered laser scan published to: /base_scan_filtered

        - Closest obstacle arrow marker published to: /closest_obstacle_marker
            -(Blue arrow with blue text displaying current distance from nearest object)

        - Wall angle text marker published to: /angle_marker
            -(needs to be pretty close to the wall to give consistent results, displays the angle in radians above robot)

    Service:
        -To test the stopping distance service run: rosservice call /stopping_distance "distance: 4.0"
        -You can replace the 4.0 with any valid float above 0.5 and it will return true, if it is below 0.5 it will see it as invalid and return false
        -It assumes you wont give it a value too large so the robot runs into a wall :)

    Action Server:
        - To test the action server use the action client node: rosrun rob599_hw1 set_goal_client.py 1.5
        - You can replace the 1.5 with any valid float above 0.5
        - It will make the fetch move to the goal and post its feedback and show the result when it has reached its location
        - Runs a little long on purpose to make sure it is fully stopped


INDIVIDUAL NODES:
    Approach node: rosrun rob599_hw1 approach.py
        - defaults to 1, if filter node is not running it will go backwards because the walls on the sides are so close but will stop once it detects 1 meter
        - subscribes to the base_scan by default, I remap this in the launch file so it listens to the filtered laser scan topic so just use the launch file

    Filter node: rosrun rob599_hw1 approach.py 
        - publishes filtered laserscan data that approach node uses when it it remapped in the launchfile

    Wall angle node: rosrun rob599_hw1 wall_angle.py 
        - publishes both a text marker in rviz and a float to another topic with the angle to the wall

    Action client test node: rosrun rob599_hw1 set_goal_client.py 1.5
        -takes a command line argument to set the distance of the robot
        -displays feedback as well as the result once completed


