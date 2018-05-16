# me134_explorer 

This is a sample ROS node for students in CS/EE/ME 134 to clone and modify to explore the simulated space.

See lab3_prelab.pdf for instructions.

This will be updated with an additional launch file once we've tested on the real robot.

In addition to what's described in the lab3_prelab.pdf I have added command line options to the me134_explorer.py.
Included in those options (see below) is a --strategy option. This allows you to change the default strategy from FindClosestFrontier (which fails because it would crash the robot into a wall) to FindRandomEmptySpace (which just finds a random empty square of 2*safety_radius_m  meters by 2*safety_radius_m meters. You are expected to write an algorithm that explores the map more efficiently than FindRandomEmptySpace.

The global_cost_map doesn't seem to update regularly so I've turned off it's plotting by default. You're free to use it if you can figure out how to read the updates.

$ rosrun me134_explorer me134_explorer.py --help
usage: me134_explorer.py [-h]
                         [--strategy {FindClosestFrontier,FindRandomEmptySpace}]
                         [--safety_radius_m SAFETY_RADIUS_M]
                         [--initial_movement_m INITIAL_MOVEMENT_M]
                         [--plot_global_costmap PLOT_GLOBAL_COSTMAP]
                         [--plot_map PLOT_MAP]

optional arguments:
  -h, --help            show this help message and exit
  --strategy {FindClosestFrontier,FindRandomEmptySpace}
  --safety_radius_m SAFETY_RADIUS_M
                        default: 0.25
  --initial_movement_m INITIAL_MOVEMENT_M
                        default: -0.25
  --plot_global_costmap PLOT_GLOBAL_COSTMAP
                        default: 0
  --plot_map PLOT_MAP   default: 1



Pull requests welcome.
