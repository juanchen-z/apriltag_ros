#!/usr/bin/env python

import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mavros_msgs.msg import OverrideRCIn, VFR_HUD
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8

def q_to_euler(q):
    e1 = q[0]
    e2 = q[1]
    e3 = q[2]
    eta = q[3]
    rot = np.array([[1 - 2 * (e2**2 + e3**2),
                    2 * (e1 * e2 - e3 * eta),
                    2 * (e1 * e3 + e2 * eta)],
                    [2 * (e1 * e2 + e3 * eta),
                    1 - 2 * (e1**2 + e3**2),
                    2 * (e2 * e3 - e1 * eta)],
                    [2 * (e1 * e3 - e2 * eta),
                    2 * (e2 * e3 + e1 * eta),
                    1 - 2 * (e1**2 + e2**2)]])
    # Roll
    roll = np.arctan2(rot[2, 1], rot[2, 2])
    # Pitch
    den   = np.sqrt(rot[2, 1]**2 + rot[2, 2]**2)
    pitch = np.arctan(-rot[2, 0]/ max(0.0001, den))
    # Yaw
    yaw = np.arctan2(rot[1, 0], rot[0, 0])

    euler = np.array([roll, pitch ,yaw])

    return euler


def odometry_msg_to_list(bag_fd, topic_id):
    p_xs = []
    p_ys = []
    p_zs = []    

    rolls = []
    pitchs = []
    yaws = []

    stamps = []

    for _, msg, _ in bag_fd.read_messages(topics=topic_id):

        qb2n = np.zeros(4)
        euler = np.zeros(3)


        stamps.append(msg.header.stamp.to_sec())
        p_xs.append(msg.pose.pose.position.x)
        p_ys.append(msg.pose.pose.position.y)
        p_zs.append(msg.pose.pose.position.z)

        qb2n = np.array([msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w])

        euler = q_to_euler(qb2n)

        rolls.append(euler[0])
        pitchs.append(euler[1])
        yaws.append(euler[2])
   
    return stamps, p_xs, p_ys, p_zs, rolls, pitchs, yaws


def main(argv):

    if len(argv) < 2:
        print("usage: python plot_data.py [bag_path]")

    bag_id = argv[1]

    bag_fd = rosbag.Bag(bag_id)

    # choose topics
    topic_id = '/filtered_odometry'
    # topic_id = '/tag_odometry'


    raw_t, raw_px, raw_py, raw_pz, rolls, pitchs, yaws = odometry_msg_to_list(bag_fd, topic_id)

    ##--------------------------------------##
    fig = plt.figure(10)
    ax = plt.axes(projection="3d")
    ax.plot3D(raw_px, raw_py, raw_pz, 'red', linewidth=2)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend(['visual measurment'])
    ax.set_title('3D trajectory')
    plt.savefig('3Dtrajectory.png', dpi = 1000)
    ##--------------------------------------##
    fig = plt.figure(11)
    ax = plt.subplot(1,1,1)
    plt.plot(raw_px, raw_py)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('2D trajectory')
    plt.savefig('2Dtrajectory.png', dpi = 1000)
    ##--------------------------------------##
    # fig = plt.figure(12)
    # ax = plt.subplot(1,1,1)
    # ax.plot(dt)
    # ax.set_xlabel
    # ax.set_ylabel('time(s)')
    # ax.set_title('time differences of measurements')
    # plt.savefig('timediff.png', dpi = 1000)
    # ##--------------------------------------##
    fig = plt.figure(20)
    ax = plt.subplot(3,1,1)
    ax.plot(raw_t,raw_px)
    ax.set_title('positions')
    ax.set_ylabel('x')
    ax = plt.subplot(3,1,2)
    ax.plot(raw_t,raw_py)  
    ax.set_xlabel('time(s)')
    ax.set_ylabel('y')
    ax = plt.subplot(3,1,3)
    ax.plot(raw_t,raw_pz)  
    ax.set_xlabel('time(s)')
    ax.set_ylabel('z')

    plt.savefig('raw meas trajectories.png', dpi = 1000)  

    # ##--------------------------------------##
    fig = plt.figure(21)
    ax = plt.subplot(3,1,1)
    ax.plot(raw_t,rolls)
    ax.set_title('euler (degrees)')
    ax.set_ylabel('roll')
    ax = plt.subplot(3,1,2)
    ax.plot(raw_t,pitchs)  
    ax.set_xlabel('time(s)')
    ax.set_ylabel('pitch')
    ax = plt.subplot(3,1,3)
    ax.plot(raw_t,yaws)  
    ax.set_xlabel('time(s)')
    ax.set_ylabel('yaw')

    plt.savefig('euler measurements.png', dpi = 1000)  

    ##--------------------------------------##
    if 0:
        fig = plt.figure(21)
        ax = plt.subplot(2,1,1)
        ax.plot(t_p,pose_n)
        ax.set_ylabel('x')
        ax = plt.subplot(2,1,2)
        ax.plot(t_p,pose_d)  
        ax.set_xlabel('time(s)')
        ax.set_ylabel('y')
        plt.savefig('fusion trajectories.png', dpi = 1000)  
    ##--------------------------------------##



    plt.show()


if __name__ == '__main__':

    main(sys.argv)
