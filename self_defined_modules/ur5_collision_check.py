from self_defined_modules.robot_class import Joint_point, Cube_obstacle          
from self_defined_modules.get_distance import *
from ur_kinematics.foward_kinematics import FK
import numpy as np
          
def ur5_collision_check(joint_point:Joint_point, cube_obstacle:Cube_obstacle) -> bool:     
          #Get the coordinate origin of DH
          [T2, T3, T4, T5, T6, T7] = FK(joint_point)
          #check link1:
          o1 = Point([0,0,0])
          o2 = Point([0,0,0.16])
          d1 = distance_line_to_cube_obstacle(o1, o2, cube_obstacle)
          if d1 <= 0.075:
                    link1_collision_hapen = True
          else:
                    link1_collision_hapen = False
          
          #check link2:
          p1 = T3*np.matrix([[0.483],[0],[0.1357],[1]])
          p2 = T3*np.matrix([[-0.058],[0],[0.1357],[1]])
          o1 = Point([float(p1[0]),float(p1[1]),float(p1[2])])
          o2 = Point([float(p2[0]),float(p2[1]),float(p2[2])])
          d2 = distance_line_to_cube_obstacle(o1, o2, cube_obstacle)
          if d2 <= 0.0872:
                    link2_collision_hapen = True
          else:
                    link2_collision_hapen = False
          
          #check link3:
          p1 = T4*np.matrix([[0.45043],[0],[0.016],[1]])
          p2 = T4*np.matrix([[-0.0375],[0],[0.016],[1]])
          o1 = Point([float(p1[0]),float(p1[1]),float(p1[2])])
          o2 = Point([float(p2[0]),float(p2[1]),float(p2[2])])
          d3 = distance_line_to_cube_obstacle(o1, o2, cube_obstacle)
          if d3 <= 0.07959:
                    link3_collision_hapen = True
          else:
                    link3_collision_hapen = False

          #check link4 + link5 + link6:
          o = Point([T5[0,3], T5[1,3], T5[2,3]])
          d4 = distance_point_to_cube_obstacle(o, cube_obstacle)
          if d4 <= 0.155:
                    link4_collision_hapen = True
          else:
                    link4_collision_hapen = False

          if link1_collision_hapen or link2_collision_hapen or link3_collision_hapen or link4_collision_hapen:
                    return True         #collision hapen
          else:
                    return False        #safe