#!/usr/bin/env python3
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image, LaserScan
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import moveit_commander
import os
import sys
from reset import GazeboTools

from random import random

class Genetic(object):

    def __init__(self, count=100, gens=50):
        self.per_gen = count
        self.gens = gens
        self.crossover_rate = 0
        self.mutation_rate = 0
        self.vel_start_min = 50
        self.vel_start_range = 100
        self.ang_start_range = 20
        self.position_start_range = 20
        #in the future, convert newtons to robot velocity

        #min starting vel: 50
        #max starting vel: 150

        #position: who fucking knows

        #angle: +- 20

        self.chroms = []
        self.rewards = []
        for i in range(self.per_gen):
            self.chroms.append([random()*self.vel_start_range+self.vel_start_min, #velocity
                                random()*self.ang_start_range*2-self.ang_start_range, #angle
                                random()*self.position_start_range*2-self.position_start_range]) #y position
            # TODO: consider making list of chroms a list of dictionaries instead of a list of lists
            self.rewards.append(-1)


        self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=60)

        return

    def crossover(self):
        #chrom 1 [100, 50, 10]
        #chrom 2 [90, 30, 20]
        #x 1 [90, 50, 20]
        #x 2 [100, 30, 10]
        return

    def mutate(self):
        return

    def test_gen(self):
        q_list = quaternion_from_euler(0, 0, 0)
        q = Quaternion()
        q.x = q_list[0]
        q.y = q_list[1]
        q.z = q_list[2]
        q.w = q_list[3]

        for i in range(self.per_gen):
            current_chrom = self.chroms[i]
            print("chrom " + str(i)+ " is "+str(self.chroms[i]))
            reward = 0


            ball = [1, 0, 0]
            rospy.sleep(0.01)
            p = Pose(position=Point(x=ball[0], y=ball[1], z=ball[2]), orientation=q)
            t = Twist(linear=Vector3(current_chrom[0]*0.2*0.2,0,0), angular=Vector3(0,0,0))
            model_state = ModelState(model_name='lane_0_ball', pose=p, twist=t)
            model_state.reference_frame = "world"
            print("calling run")
            node = GazeboTools(0)
            parameters = {
              "robot_speed": current_chrom[0]*0.2*0.2,
              "robot_time": current_chrom[1]*0.2*0.2
            }
            reward = node.run(parameters)
            rospy.sleep(0.5)

            print("fininshed chrom " + str(i))

    def run(self):
        print("running")
        self.test_gen()
        print("gen tested")
        #determine survivors
        #pair parentS
        #self.crossover() #CHANCE
        #self.mutuate() #CHANCE
        #reset rewards
        #test again
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("genetic")
    gen = Genetic()
    gen.run()
