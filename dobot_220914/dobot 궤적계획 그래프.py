import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

def FowardKinematics(J1,J2,J3):

    L1 = 138
    L2 = 142
    L3 = 160
    L4 = 46 #mm

    th1 = J2
    th2 = J3
    th3 = J1

    C1 = math.cos(math.radians(th1))
    C2 = math.cos(math.radians(th2))
    C3 = math.cos(math.radians(th3))
    S1 = math.sin(math.radians(th1))
    S2 = math.sin(math.radians(th2))
    S3 = math.sin(math.radians(th3))

    x = C1 * (L2 * S2 + L3 * C3 + L4) 
    y = S1 * (L2 * S2 + L3 * C3 + L4)
    z = L2 * C2 - L3 * S3

    return x,y,z

def InversKinematics(x,y,z):

    #L1 = 138
    L1 = 142
    L2 = 160
    d = 46

    r = math.sqrt(x**2 + y**2)
    p = math.sqrt((r-d)**2 + z**2)
    alpha = math.degrees(math.acos((L1**2 + p**2 - L2**2)/(2*L1*p)))
    beta = math.degrees(math.atan(z/(r-d)))
    gama = math.degrees(math.acos((L1**2+L2**2-p**2)/(2*L1*L2)))

    th1 = math.degrees(math.atan(y/x))
    th2 = 90 - beta - alpha
    th3 = 90 + th2 - gama

    return th1,th2,th3

def Trajectory_planning(t,th0,thv,thf):

    tf = 1.5
    tv = 0.3

    if t < tv:

        #제 1경로 계수
        a10 = th0
        a11 = 0
        a12 = (12*thv - 3*thf - 9*th0)/(4 * tf**2)
        a13 = (-8*thv + 3*thf + 5*th0)/(4 * tf**3)

        th1_result = a10 + a11 * t + a12 * t**2 + a13 * t**3
        
        return th1_result

    elif tv <= t:

        t = t - tv

        #제2 경로 계수
        a20 = thv
        a21 = (3*thf - 3*th0)/(4*tf)
        a22 = (-12*thv + 6*thf + 6*th0)/(4* tf**2)
        a23 = (8*thv - 5*thf - 3*th0)/(4* tf**3)

        th2_result = a20 + a21 * t + a22 * t**3 + a23 * t**3
        
        return th2_result
    
def main():

    mode = int(input('mode - Fk: 0, Ik : 1 '))

    if mode == 0 :

        J1 = float(input('J1: '))
        J2 = float(input('J2: '))
        J3 = float(input('J3: '))

        x,y,z = FowardKinematics(J1,J2,J3)

        print("x: %f, y: %f, z: %f" %(x,y,z))

    elif mode == 1:

        x = float(input('x: '))
        y = float(input('y: '))
        z = float(input('z: '))

        th1,th2,th3 = InversKinematics(x,y,z)

        print("th1: %f, th2: %f, th3: %f" %(th1,th2,th3))

def main2():

    point_num = 5 # 궤적에 위치하는 점의 수

    start_point_joint = [24.3,20.0838,50.26,-80] # 시작 위치의 joint parameter [ j1,j2,j3,j4]
    pass_point_joint = [-15.3,11.05,38.86,-80.8] # 경유 지점 joint parameter
    end_point_joint = [-54.18,-0.86,14.39,-66.79] # 도착점의 joint parameter

    start_point_x,start_point_y,start_point_z = FowardKinematics( float(start_point_joint[0]), float(start_point_joint[1]), float(start_point_joint[2]) )
    pass_point_x,pass_point_y,pass_point_z = FowardKinematics( float(pass_point_joint[0]), float(pass_point_joint[1]), float(pass_point_joint[2]) )
    end_point_x,end_point_y,end_point_z = FowardKinematics( float(end_point_joint[0]), float(end_point_joint[1]), float(end_point_joint[2]) )

    Joint_trajectory = [] # 궤적의 점들의 joint parameter
    Point_trajectory = [] # 궤적의 점들의 x,y,z

    plt_x = []
    plt_y = []
    plt_z = []

    for i in range(point_num+1):

        point_joint_parameter = [] # 각점의 joint parametr [j1,j2,j3,j4]

        step_time = 1.5 / point_num # 각 점 사이의 시간

        t = step_time * i

        print("t: ",t)

        for j in range(4):

            joint = Trajectory_planning(t, float(start_point_joint[j]), float(pass_point_joint[j]), float(end_point_joint[j]))

            point_joint_parameter.append(joint)            

        Joint_trajectory.append(point_joint_parameter)

        print(point_joint_parameter)

        x,y,z = FowardKinematics( float(point_joint_parameter[0]), float(point_joint_parameter[1]), float(point_joint_parameter[2]) ) # 궤적의 각 점들의 x,y,z

        points = [x,y,z]

        plt_x.append(x)
        plt_y.append(y)
        plt_z.append(z)

        Point_trajectory.append(points)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    fontlabel = {"fontsize":"large", "color":"gray", "fontweight":"bold"}
    ax.set_xlabel("X", fontdict=fontlabel, labelpad=12)
    ax.set_ylabel("Y", fontdict=fontlabel, labelpad=12)
    ax.set_title("Z", fontdict=fontlabel)

    ax.plot(plt_x,plt_y,plt_z, color = 'b', alpha = 0.2)
    ax.scatter(plt_x,plt_y,plt_z, color = 'b', alpha = 0.2)
    ax.scatter(start_point_x,start_point_y,start_point_z,color = 'r', alpha = 0.5)
    ax.scatter(pass_point_x,pass_point_y,pass_point_z,color = 'r', alpha = 0.5)
    ax.scatter(end_point_x,end_point_y,end_point_z,color = 'r', alpha = 0.5)

    plt.show()

main2()
