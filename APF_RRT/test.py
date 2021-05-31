from APF_RRT.Artifial_potential_field_RRT import APF_RRT
from self_defined_modules.robot_class import Cube_obstacle, Joint_point, Point
from math import pi

start = Joint_point([0]*6)
goal = Joint_point([pi/2]+[0]*5)
cube_obstacle = Cube_obstacle(Point([-0.6, -0.6, 0.15]), 0.3, 0.3, 0.3)

apf_rrt = APF_RRT(start, goal, cube_obstacle)
apf_rrt.path_planning()