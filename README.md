# CMP9767M Assignment - Bethan Moncur

## Summary of the solution

The system loads a map already created using Gmapping (http://wiki.ros.org/gmapping). Thorvald_001 orientates itself using adaptive Monte Carlo localisation (http://wiki.ros.org/amcl) whilst moving using odometry-based targets. The system then utilises topological navigation (http://wiki.ros.org/topological_navigation) – at each topological node, it performs colour thresholding and contour detection to detect grape bunches (https://opencv.org/). To avoid double counting, the centroid of the grape bunch contour is converted to map coordinates using tf (http://wiki.ros.org/tf) and stored in a list, which is checked for proximity to each subsequent grape contour. At the end of the grape row, the count is reset and the counting process is repeated on the other side of the row to obtain an average. The system is orchestrated using a control node to communicate between the counting and navigation nodes.

## Key files
* `uol_cmp9767m_tutorial/scripts/grape_control.py` - script for the control node of the system
* `uol_cmp9767m_tutorial/scripts/grape_localise.py` - script for amcl localisation process at the start
* `uol_cmp9767m_tutorial/scripts/grape_track.py` - script to perform image analysis and grape counting
* `uol_cmp9767m_tutorial/scripts/set_grape_route_goal.py` - script to send topological navigation goals for the robot

* `uol_cmp9767m_tutorial/maps/grape_route.yaml`- map to define the topological nodes and edges

* `uol_cmp9767m_tutorial/launch/grape.launch` - launch file to launch the simulation and nodes

## How to start the system

* Boot in Ubuntu 18.04
* Install the required packages:

    ```
    sudo apt-get update && sudo apt-get upgrade
    ```
    
    ```
    sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop
    ```
    
    ```
    sudo apt-get install \
        ros-melodic-opencv-apps \
        ros-melodic-rqt-image-view \
        ros-melodic-uol-cmp9767m-base \
        ros-melodic-find-object-2d \
        ros-melodic-video-stream-opencv \
        ros-melodic-topic-tools \
        ros-melodic-rqt-tf-tree
        ros-melodic-opencv-apps \
        ros-melodic-rqt-image-view \
        ros-melodic-image-geometry \
        ros-melodic-uol-cmp9767m-base \
        ros-melodic-uol-cmp9767m-tutorial \
        ros-melodic-find-object-2d \
        ros-melodic-video-stream-opencv \
        ros-melodic-image-view
        ros-melodic-robot-localization \
        ros-melodic-thorvald \
        ros-melodic-velodyne-description \
        ros-melodic-kinect2-description \
        ros-melodic-topological-navigation \
        ros-melodic-teleop-tools \
        ros-melodic-amcl
        ros-melodic-robot-localization \
        ros-melodic-topological-navigation \
        ros-melodic-amcl \
        ros-melodic-fake-localization \
        ros-melodic-carrot-planner
        ros-melodic-topological-utils \
        ros-melodic-topological-navigation \
        ros-melodic-topological-navigation-msgs \
        ros-melodic-strands-navigation
        ros-melodic-gmapping
    ```
    
    ```
    source /opt/ros/melodic/setup.bash
    ```
    
* Create and build a catkin workspace

    ```
    mkdir -p ~/catkin_ws/src
    ```
    
    ```
    cd ~/catkin_ws/
    ```
    
    ```
    catkin_make
    ```

    ```
    source devel/setup.bash
    ```
    
* Clone the git-repository into the catkin workspace - change directory to `catkin_ws/src`, followed by:

    ```
    git clone https://github.com/bethanmoncur/CMP9767M.git
    ```
    ```
    catkin_make
    ```
    
    ```
    source devel/setup.bash
    ```
    
* In your home directory, create an empty folder called `mongodb`
* Launch the simulation:
    ```
    roslaunch uol_cmp9767m_tutorial grape.launch
    ```
    
* In a new terminal, navigate to the catkin workspace. Source the workspace and load the map file:
    ```
    source devel/setup.bash
    ```
    
    ```
    rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/grape_route.yaml
    ```
    
* In rviz, open the `grape_config.rviz` visualisation file: `File -> Open Config -> catkin_ws -> src -> uol_cmp9767m_tutorial -> config -> grape_config.rviz`
* In a new terminal window, source the workspace and run the localisation sequence:
    ```
    source devel/setup.bash
    ```

    ```
    rosrun uol_cmp9767m_tutorial grape_localise.py
    ```
    
The script will ask: `Does the robot need localising? [y/n]`
At this point, open the rviz window and check if the robot is localised -  laser readings aligned with the map (edges aligned with the walls and dots above the black dots) and red amcl particles in a small bunch under the robot.

* If the robot is localised, in the terminal, type `n` followed by `enter`. This will tell the robot to begin the grape counting sequence.

* If the robot is not localised, type `y` followed by `enter`. This will tell the robot to navigate to a near-by point (using odometry) to localise itself. The script will ask: `Has the robot successfully localised itself? [y/n]`

    * If the robot is localised, in the terminal, type `y` followed by `enter`. This will tell the robot to begin the grape counting sequence.
    * If the robot is not localised, type `n` followed by `enter`. This will refresh the localisation particle estimates and repeat the previous step. Repeat this process until the robot is localised.

    Once the robot is localised and has started the grape counting process, the terminal window running the simulation will start printing information about the status of the system, such as navigation status between topological nodes and the status of the grape counting system.

    The number of grape bunches counted during each iteration of the process will be printed along with the coordinates of the grape centroid. It also prints the number of grape bunches from the previous count. Once the number of grapes counted does not change between iterations (i.e, current count is the same as the previous count), the robot will continue onto the next topological node.

    Once it reaches the end of the row, the grape count will be printed along with the average count (this will be the same as the grape count for the first counting run). It then resets the count and repeats the counting process from the other side of the row. The system will print a list containing the count for each run, and the average of the runs.

To repeat the counting process (to obtain more run values to increase accuracy of the counting process), re-run the localisation node:

```
rosrun uol_cmp9767m_tutorial grape_localise.py
```
    
* When the script asks: `Does the robot need localising? [y/n]` type `n` followed by `enter` because Thorvald_001 will still be localised.

To get the grape count separate to the system status updates, subscribe to the `/grape_count` topic in a new terminal window. This publishes the number of grapes from the current run down the row (not the average).

```
rostopic echo /grape_count
```
