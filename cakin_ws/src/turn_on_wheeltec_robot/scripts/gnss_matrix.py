#!/usr/bin/env python
# encoding: utf-8
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import message_filters

def data_callback(gnss_msg,ndt_msg):
    global ndt_pose,gnss_pose,count
    count+=1    
    gnss_stamp = gnss_msg.header.stamp
    ndt_stamp = ndt_msg.header.stamp
    gnss_t1 = gnss_stamp.secs * 1000000000 + gnss_stamp.nsecs
    #ndt_t1 = ndt_stamp.secs * 1000000000 + ndt_stamp.nsecs  
    #print("gnss_t1 is ",gnss_t1)
    print("count is ",count)     
    gnss_data = {'nsec': gnss_t1, 'x': gnss_msg.pose.position.x, 'y': gnss_msg.pose.position.y, 'z': gnss_msg.pose.position.z}
    ndt_data = {'nsec': gnss_t1, 'x': ndt_msg.pose.position.x, 'y': ndt_msg.pose.position.y, 'z': ndt_msg.pose.position.z}

    gnss_pose.append(gnss_data)
    ndt_pose.append(ndt_data)
    if len(gnss_pose) > 500 and len(ndt_pose) > 500:
        matrix_computation() 
        
def gnss_callback(msg):
    global gnss_pose
    stamp = msg.header.stamp
    t1 = stamp.secs * 1000000000 + stamp.nsecs
    data = {'nsec': t1, 'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z}
    gnss_pose.append(data)
    #print("gnss_pose is ",len(gnss_pose))
def ndt_callback(msg):
    global ndt_pose
    stamp = msg.header.stamp
    t1 = stamp.secs * 1000000000 + stamp.nsecs
    data = {'nsec': t1, 'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z}
    ndt_pose.append(data)
    print("ndt_pose is ",len(ndt_pose)) 
    if len(gnss_pose) > 500 and len(ndt_pose) > 500:
        print("matrix_computation")
        matrix_computation()
              
def matrix_computation():
    global ndt_pose
    global gnss_pose
    gnss_matrix = []
    ndt_matrix = []
    sample_num=70
    print("matrix_computation")
    for i in range(sample_num):
        n1 = (i + 1) * 5
        #print("n1 is ",n1)
        #if n1 < len(gnss_pose):
        t1 = gnss_pose[n1]['nsec']       
        index = 0
        for j in range(index, len(ndt_pose)):
            if -30000000 < t1 - ndt_pose[j]['nsec'] < 30000000:
                gnss_c = gnss_pose[n1]
                ndt_c = ndt_pose[j]
                gnss_matrix.append([gnss_c['x'], gnss_c['y'], gnss_c['z']])
                ndt_matrix.append([ndt_c['x'], ndt_c['y'], ndt_c['z']])
                index = j
                break
    print('gnss_matrix matrix:')
    print ""
    print gnss_matrix
    print('ndt_matrix matrix:')
    print ""
    print ndt_matrix    
    gnss_matrix = np.array(gnss_matrix).reshape((-1, 3))
    ndt_matrix = np.array(ndt_matrix).reshape((-1, 3))

    data = np.hstack((gnss_matrix, ndt_matrix))
    np.random.shuffle(data)
    gnss_matrix = data[:, :3]
    ndt_matrix = data[:, 3:]

    test_a = ndt_matrix[-10:]
    test_b = gnss_matrix[-10:]
    a = ndt_matrix[:-10]
    b = gnss_matrix[:-10]

    r, t = rigid_transform_3D(b, a)
    print('r matrix:')
    print("")
    print(r)
    print('t matrix:')
    print("")
    print(t)
    print("")
    print("")
    rospy.signal_shutdown("Finished processing")

def compute_matrix():
    global gnss_pose, ndt_pose,count,tmp
    count=0
    tmp=0
    gnss_pose = []
    ndt_pose = []
    gnss_matrix = []
    ndt_matrix = []
    sample_num = 70

    rospy.init_node('matrix_computation_node')
    rospy.Subscriber('/gnss_pose', PoseStamped, gnss_callback)
    rospy.Subscriber('/ndt_pose', PoseStamped, ndt_callback)
    
    #gnss_sub = message_filters.Subscriber('/gnss_pose', PoseStamped)
    #ndt_sub = message_filters.Subscriber('/ndt_pose', PoseStamped)
    #timeSynchronizer = message_filters.ApproximateTimeSynchronizer([gnss_sub, ndt_sub], 10, 0.5)
    #timeSynchronizer.registerCallback(data_callback)
    rospy.spin()
    
def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA),BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.matmul(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T,U.T)

    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
    return R, t
if __name__ == '__main__':
    compute_matrix()
   
