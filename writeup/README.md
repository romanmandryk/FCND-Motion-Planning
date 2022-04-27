Writeup of the solution for the 3D motion planning
===

1. Explain the Starter Code
------

Comparing Backyard_flyer and motion_planning files

There are issues reported in GH and Udacity knowledge which make it hard to fly a drone as it's stuck "in the building"
https://github.com/udacity/fcnd-issue-reports/issues/392
https://knowledge.udacity.com/questions/837815
After using specific version of the simulator 0.1.0 for motion planning only there is a way to fly a drone "through the roof" but landing transition never finishes 
as the drone lands on the roof instead on the ground.


Motion_planning has an extra state PLANNING between ARMING and TAKEOFF - as this can take significant time - 
and a corresponding transition between states in functions `state_callback` and `plan_path`.
It also have 2 new methods `send_waypoints` (for visualisations) and `plan_path` - which has a skeleton of useful prep work for planning and TODOs to finish the project.

planning_utils contains useful methods which were re-used few times in previous excercises like:
 - create_grid - reads provided csv file with obstacles and and creates a 2D np.array grid for a specific altitude and other metadata
 - a_star - grid based a_star algorithm with valid movements in 4 directions
 - valid_actions - function which returns only 4 directions of movement NEDU
 - heuristic - euclidian distance between 2 points (it is used in a_star algorithm)

plan_path function:
 - sets important parameters for the planning like altitude, safety distance from obstacles, start and goal positions
 - then executes a_star to find the path from start to goal and sends waypoints to the simulator for visualisation.
