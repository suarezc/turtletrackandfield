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

import concurrent.futures

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

    def run_chrom(self, chrom, lane):
        node = GazeboTools(lane)
        parameters = {
          "robot_speed": chrom[0]*0.2*0.2,
          "robot_time": chrom[1]
        }
        reward = node.run(parameters)
        return reward, lane

    def test_gen(self):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for j in range(self.per_gen // 10):
                futures = []
                for i in range(10):
                    futures.append(executor.submit(self.run_chrom, chrom=self.chroms[j*10 + i], lane=i))
                for future in concurrent.futures.as_completed(futures):
                    reward, lane = future.result()
                    print("lane ", lane, "got", reward, "points")

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
