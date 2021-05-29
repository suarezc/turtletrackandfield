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

import random
from datetime import datetime
# Crossovers: linear crossover, simulated binary crossover
# Mutations: Scalar mutation, vary in magnitude and number of chromosomes affected, substitution
# select survivors, which percentile we select, how we allocate to crossover, mutate,
  
class Genetic(object):

    def __init__(self, count=20, gens=30):
        self.per_gen = count
        self.gens = gens
        self.crossover_rate = 0
        self.mutation_rate = 0
        self.vel_start_min = 2
        self.vel_start_range = 4
        self.time_range = 7
        self.ang_start_range = 20
        self.position_start_range = 20
        self.keep =  []#higest 25
        self.parents = [] #middle 50
        self.mutation_only = [] #lowest 25
        #in the future, convert newtons to robot velocity

        #min starting vel: 50
        #max starting vel: 150

        #position: who fucking knows

        #angle: +- 20


        self.chroms = []
        for i in range(self.per_gen):
            self.chroms.append([random.random()*self.vel_start_range+self.vel_start_min, #velocity
                                random.random()*self.time_range, #time
                                random.random()*self.ang_start_range*2-self.ang_start_range, #angle
                                random.random()*self.position_start_range*2-self.position_start_range, #y position
                                -1]) #reward always last index
            # TODO: consider making list of chroms a list of dictionaries instead of a list of lists


        self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=60)

        return

    #Does not use crossover_rate
    def linear_crossover(self):
        random.shuffle(self.parents)
        length = len(self.parents)//2

        for i in range(length):
            weight_one = random.random()
            weight_two = random.random()

            child_one = [self.parents[i][j]*weight_one + self.parents[i+length][j]*(1- weight_one) for j in range(5)]
            child_two = [self.parents[i][j]*weight_two + self.parents[i+length][j]*(1- weight_two) for j in range(5)]
            self.parents[i] = child_one
            self.parents[i + length] = child_two

        return

    #does not use mutation_rate
    def scalar_mutate(self):

        for chrom in self.mutation_only:
            chrom = [val * (random.random() + .5) for val in chrom]
    
        return

    def run_chrom(self, chrom, lane):
        node = GazeboTools(lane)
        parameters = {
          "robot_speed": chrom[0],
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
                    self.chroms[j*10 + lane][-1] = reward
                    print("lane ", lane, "got", reward, "points")

    def sort_and_process_chroms(self):
        self.chroms.sort(key = lambda reward: reward[-1])
       
        for i, chrom in enumerate(self.chroms):
            if i < 4:
                self.mutation_only.append(chrom)
            elif i >= 16:
                self.keep.append(chrom)
            else:
                self.parents.append(chrom)
        

        self.linear_crossover()
        self.scalar_mutate()

        print("keep length", len(self.keep))
        print("parents length", len(self.parents))
        print("mutation length", len(self.mutation_only))

        self.chroms =  self.keep + self.parents + self.mutation_only
        print("chroms length", len(self.chroms))

        #reset reward
        for chrom in self.chroms:
            chrom[-1] = -1

        return


    def run(self):
        while not rospy.is_shutdown() and self.gens != 0:            
            print("running " + str(self.gens))
            t_start = datetime.now()
            self.test_gen() #done
            t_end = datetime.now()
            print(t_end - t_start)
            print("gen tested")
            print("gen stats")
            avg_speed = 0
            avg_time = 0
            avg_reward = 0

            for chrom in self.chroms:
                avg_speed += chrom[0]
                avg_time += chrom[1]
                avg_reward += chrom[-1]
            
            print(avg_speed/20, avg_time/20, avg_reward/20)

            self.sort_and_process_chroms()
            self.keep = []
            self.parents = []
            self.mutation_only = []

            self.gens -= 1

        #determine survivors # score-based
        #pair parents #random
        #self.crossover() #CHANCE
        #self.mutuate() #CHANCE
        #reset rewards
        #test again


if __name__ == "__main__":
    rospy.init_node("genetic")
    gen = Genetic()
    gen.run()
