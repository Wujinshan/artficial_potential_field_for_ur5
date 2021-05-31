from self_defined_modules.robot_class import Point, Cube_obstacle
import numpy.linalg

"""
This document includes some distance calculate functions
"""
def distance_point_to_point(p1:Point, p2:Point) -> float:
          """
          Calculate the distance between p1 and p2
          """
          return numpy.linalg.norm(p1.vector-p2.vector)

def distance_point_to_cube_obstacle(p:Point, cube_obstacle:Cube_obstacle) ->float:
          """
          Calculate the distance between p and cube_obstacle
          """
          x = p.x
          y = p.y
          z = p.z

          x_min = cube_obstacle.x - cube_obstacle.w/2
          x_max = cube_obstacle.x + cube_obstacle.w/2
          y_min = cube_obstacle.y - cube_obstacle.l/2
          y_max = cube_obstacle.y + cube_obstacle.l/2
          z_min = cube_obstacle.z - cube_obstacle.h/2
          z_max = cube_obstacle.z + cube_obstacle.h/2

          point_z1 = Point([x_min, y_min, z])
          point_z2 = Point([x_min, y_max, z])
          point_z3 = Point([x_max, y_min, z])
          point_z4 = Point([x_max, y_max, z])
          point_x1 = Point([x, y_min, z_min])
          point_x2 = Point([x, y_min, z_max])
          point_x3 = Point([x, y_max, z_min])
          point_x4 = Point([x, y_max, z_max])
          point_y1 = Point([x_min, y, z_min])
          point_y2 = Point([x_min, y, z_max])
          point_y3 = Point([x_max, y, z_min])
          point_y4 = Point([x_max, y, z_max])

          #vertex
          p1 = Point([x_min, y_min, z_min])
          p2 = Point([x_min, y_min, z_max])
          p3 = Point([x_min, y_max, z_min])
          p4 = Point([x_min, y_max, z_max])
          p5 = Point([x_max, y_min, z_min])
          p6 = Point([x_max, y_min, z_max])
          p7 = Point([x_max, y_max, z_min])
          p8 = Point([x_max, y_max, z_max])

    
          #situation 1:
          if x >= x_min and x <= x_max and y >= y_min and y <= y_max and z >= z_min and z <= z_max:
                    distance = 0
          #situation 2:
          if x >= x_min and x <= x_max and y >= y_min and y <= y_max and (z <= z_min or z >= z_max):
                    distance = min(abs(z-z_max),abs(z-z_min))
          if x >= x_min and x <= x_max and z >= z_min and z <= z_max and (y <= y_min or y >= y_max):
                    distance = min(abs(y-y_max),abs(y-y_min))
          if z >= z_min and z <= z_max and y >= y_min and y <= y_max and (x <= x_min or x >= x_max):
                    distance = min(abs(x-x_max),abs(x-x_min))
          #situation 3:
          if (x <= x_min or x >= x_max) and (y <= y_min or y >= y_max) and z >= z_min and z <= z_max:
                    d1 = distance_point_to_point(point_z1, p)
                    d2 = distance_point_to_point(point_z2, p)
                    d3 = distance_point_to_point(point_z3, p)
                    d4 = distance_point_to_point(point_z4, p)
                    distance = min(d1, d2, d3, d4)
          if (x <= x_min or x >= x_max) and (z <= z_min or z >= z_max) and y >= y_min and y <= y_max:
                    d1 = distance_point_to_point(point_y1, p)
                    d2 = distance_point_to_point(point_y2, p)
                    d3 = distance_point_to_point(point_y3, p)
                    d4 = distance_point_to_point(point_y4, p)
                    distance = min(d1, d2, d3, d4)
          if (z <= z_min or z >= z_max) and (y <= y_min or y >= y_max) and x >= x_min and x <= x_max:
                    d1 = distance_point_to_point(point_x1, p)
                    d2 = distance_point_to_point(point_x2, p)
                    d3 = distance_point_to_point(point_x3, p)
                    d4 = distance_point_to_point(point_x4, p)
                    distance = min(d1, d2, d3, d4)
          #situation 4:
          if (x <= x_min or x >= x_max) and (y <= y_min or y >= y_max) and (z <= z_min or z >= z_max):
                    d1 = distance_point_to_point(p1, p)
                    d2 = distance_point_to_point(p2, p)
                    d3 = distance_point_to_point(p3, p)
                    d4 = distance_point_to_point(p4, p)
                    d5 = distance_point_to_point(p5, p)
                    d6 = distance_point_to_point(p6, p)
                    d7 = distance_point_to_point(p7, p)
                    d8 = distance_point_to_point(p8, p)
                    distance = min(d1, d2, d3, d4, d5, d6, d7, d8)
          return distance

def distance_line_to_cube_obstacle(p1:Point, p2:Point, cube_obstacle:Cube_obstacle) ->float:
          """
          Calculate the distance between line segment and cube_obstacle
          p1, p2 : The ends of line segment
          """
          #Divide the line segment into s segments
          s = 20
          #Get the s+1 points in the line segment
          point_list = list()
          for i in range(s+1):
                    point_list.append(Point([p1.x+i*(p2.x-p1.x)/s, p1.y+i*(p2.y-p1.y)/s, p1.z+i*(p2.z-p1.z)/s]))
          #Get distance between every point and cube obstacle
          dis = list()
          for p in point_list:
                    dis.append(distance_point_to_cube_obstacle(p,cube_obstacle))
          return min(dis)