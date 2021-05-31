from typing import List
import ur_kinematics.coordinate_transformation_functions as ctf
import ur_kinematics.ur_DH_config
from self_defined_modules.robot_class import Joint_point

#Get the DH parameters of ur5
a = ur_kinematics.ur_DH_config.UR5_DH_param.a
alpha = ur_kinematics.ur_DH_config.UR5_DH_param.alpha
d = ur_kinematics.ur_DH_config.UR5_DH_param.d

def FK(joint_point:Joint_point) -> List:
          theta = joint_point.theta
          T12 = ctf.rotz(theta[0])*ctf.trans(0,0,d[0])*ctf.rotx(alpha[0])*ctf.trans(a[0],0,0)
          T23 = ctf.rotz(theta[1])*ctf.trans(0,0,d[1])*ctf.rotx(alpha[1])*ctf.trans(a[1],0,0)
          T34 = ctf.rotz(theta[2])*ctf.trans(0,0,d[2])*ctf.rotx(alpha[2])*ctf.trans(a[2],0,0)
          T45 = ctf.rotz(theta[3])*ctf.trans(0,0,d[3])*ctf.rotx(alpha[3])*ctf.trans(a[3],0,0)
          T56 = ctf.rotz(theta[4])*ctf.trans(0,0,d[4])*ctf.rotx(alpha[4])*ctf.trans(a[4],0,0)
          T67 = ctf.rotz(theta[5])*ctf.trans(0,0,d[5])*ctf.rotx(alpha[5])*ctf.trans(a[5],0,0)

          T2 = T12
          T3 = T12*T23
          T4 = T12*T23*T34
          T5 = T12*T23*T34*T45
          T6 = T12*T23*T34*T45*T56
          T7 = T12*T23*T34*T45*T56*T67
          return([T2, T3 ,T4 ,T5, T6, T7])