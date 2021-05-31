from typing import List
from self_defined_modules.robot_class import Cube_obstacle, Point, Joint_point
from ur_kinematics.foward_kinematics import FK
from self_defined_modules.get_distance import *
from math import pi
from self_defined_modules.ur5_collision_check import ur5_collision_check

class APF_RRT():
          """
          Artificial Poential Field and RRT
          """
          def __init__(self, start:Joint_point, goal:Joint_point, cube_obstacle:Cube_obstacle) -> None:
                    self.start = start
                    self.goal = goal
                    self.cube_obstacle = cube_obstacle

                    #Get the goal position in Cartesian space
                    T = FK(self.goal)
                    self.goal_point = Point([T[5][0,3], T[5][1,3], T[5][2,3]])

                    #Set the Katt and Krep
                    self.Katt = 1
                    self.Krep = 0.1

                    #Set the range of repulsive potential field
                    self.d0 = 0.3

                    #Set the step length
                    self.step_length = 2*pi/180

          def get_attractive_energy(self, joint_point:Joint_point) -> float:
                    """
                    Get the attractive energy in joint_point
                    """
                    #Get the endlink position of theta
                    T = FK(joint_point)
                    endlink_point = Point([T[5][0,3], T[5][1,3], T[5][2,3]])

                    #Get the distance between endlink and goal
                    d = distance_point_to_point(endlink_point, self.goal_point)

                    #Return the attractive energy
                    return 0.5*self.Katt*(d**2)

          def get_repulsive_energy(self, joint_point:Joint_point) -> float:
                    """
                    Get the repulsive energy in joint_point
                    """
                    #Get the repulsive energy of each DH joints
                    Ui = list()         #Stores the repulsive energy of each DH joints
                    T = FK(joint_point)
                    for A in T:
                              DH_joint_point = Point([A[0,3], A[1,3], A[2,3]])
                              d = distance_point_to_cube_obstacle(DH_joint_point, self.get_obstacle())
                              if d < self.d0:
                                        Ui.append(0.5*self.Krep*(1/d-1/self.d0))
                              else:
                                        Ui.append(0)
                    
                    return sum(Ui)

          def get_obstacle(self) -> Cube_obstacle:
                    """
                    This is a interface to get the information of obstacle
                    """
                    return self.cube_obstacle

          def deal_with_local_minimum(self) -> List[float]:
                    """
                    Method to deal with algorithms getting stuck in local minimum
                    """
          def path_planning(self) -> List[Joint_point]:
                    """
                    Start the path planning
                    """
                    #Initial theta
                    theta = self.start.theta
                    #Initial the path
                    path = [self.start]
                    while True:
                              """
                              Initial the U and theta_list
                              U : Stores the total potential energy of once search
                              joint_point_list : Stores the joint_point of once search
                              """
                              U = []
                              joint_point_list = []
                              for theta1 in [theta[0]-self.step_length, theta[0], theta[0]+self.step_length]:
                                        for theta2 in [theta[1]-self.step_length, theta[1], theta[1]+self.step_length]:
                                                  for theta3 in [theta[2]-self.step_length, theta[2], theta[2]+self.step_length]:
                                                            for theta4 in [theta[3]-self.step_length, theta[3], theta[3]+self.step_length]:
                                                                      for theta5 in [theta[4]-self.step_length, theta[4], theta[4]+self.step_length]:
                                                                                theta6 = self.start.theta[5]
                                                                                joint_point = Joint_point([theta1, theta2, theta3, theta4, theta5, theta6])
                                                                                #Check collision
                                                                                if ur5_collision_check(joint_point, self.get_obstacle()):
                                                                                          pass
                                                                                else:
                                                                                          #Get the atrractive energy
                                                                                          Uatt = self.get_attractive_energy(joint_point)
                                                                                          #Get the repulsive energy
                                                                                          Urep = self.get_repulsive_energy(joint_point)
                                                                                          U.append(Uatt+Urep)           #The total potential energy
                                                                                          joint_point_list.append(joint_point)
                              #FInd the index of minimum U
                              index = U.index(min(U))
                              #Add the theta_list[index] into path
                              path.append(joint_point_list[index])
                              print(path[-1].theta)
                              #Update the theta
                              theta = joint_point_list[index].theta
                              #Check the exit loop condition
                              if path[-1].theta == path[-2].theta:
                                        break
                    return path
