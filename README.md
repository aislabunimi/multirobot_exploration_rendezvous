# multirobot_exploration_rendezvous
In the launch folder there are some launch files to run specific configurations of robots, environment and path planning method.

![Example](https://github.com/aislabunimi/multirobot_exploration_rendezvous/blob/journal/run_example.png)

## Dependencies
Packages to install:
```
pip3 install shapely
sudo apt install ros-noetic-costmap-2d* ros-noetic-move-base* ros-noetic-turtlebot3* ros-noetic-gmapping ros-noetic-navfn* ros-noetic-dwa-local-planner ros-noetic-global-planner
cd data
sqlite3 data_test.db
.read schema.sql
.quit
```
