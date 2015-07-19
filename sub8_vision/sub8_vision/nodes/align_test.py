#!/usr/bin/python
## Math
import numpy as np
## Display
import pygame
import time
import math
## Ros
import rospy
from tf import transformations as tf_trans
## Ros Msgs
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion

SCREEN_DIM = (640, 480)
ORIGIN = np.array([SCREEN_DIM[0]/2.0, SCREEN_DIM[1]/2.0])

def main():

    rospy.init_node("align_simulation")
    display = pygame.display.set_mode(SCREEN_DIM)
    pygame.display.set_caption("align_simulation")
    des_pose_pub = rospy.Publisher('align_test_point', Point, queue_size=1)

    #des_pose_pub_base = rospy.Publisher('base_des_pose', PointStamped, queue_size=1)

    def publish_des_pos((x, y, z)):
        '''Publish desired position of the arm end-effector based on click position'''
        des_pose_pub.publish(Point(
                    x=x, 
                    y=y, 
                    z=z,  
            )
        )

    clock = pygame.time.Clock()

    target_point = np.array([0.0, 0.0, 0.0])
    extent = 0.0
    base_angle = 0.0
    z = 0.0
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

            if event.type == pygame.KEYDOWN:
                if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q):
                    return

                if event.key == pygame.K_a:
                    pt = pygame.mouse.get_pos()
                    publish_des_pos((pt[0], pt[1], z))

        t = time.time()
        pygame.display.update()
        clock.tick(20)
        display.fill((0, 0, 0))

if __name__ == '__main__':
    main()