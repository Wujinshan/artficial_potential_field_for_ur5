from typing import List
import numpy as np

class Point():
          """
          Represent a point in Cartesian space
          """
          def __init__(self, p:List[float]) -> None:
                    """
                    p : The coordinate of the point in Cartesian space
                    """
                    self.p = p
                    self.x = p[0]
                    self.y = p[1]
                    self.z = p[2]
                    self.vector = np.array(p)       #Vector calculation

class Joint_point():
          """
          Represent a point in joint space
          """
          def __init__(self, theta:List[float]) -> None:
                    """
                    theta : The coordinate of the point in joint space
                    """
                    self.theta = theta
                    self.theta1 = theta[0]
                    self.theta2 = theta[1]
                    self.theta3 = theta[2]
                    self.theta4 = theta[3]
                    self.theta5 = theta[4]
                    self.theta6 = theta[5]
                    self.vector = np.array(theta)

class Cube_obstacle():
          """
          Represent a cube obstacle
          """
          def __init__(self, center:Point, l:float, w:float, h:float) -> None:
                    """
                    center : The center coordinate of cube obstacle
                    l : length          w : width           h : high
                    """
                    self.x = center.x
                    self.y = center.y
                    self.z = center.z
                    self.l = l
                    self.w = w
                    self.h = h