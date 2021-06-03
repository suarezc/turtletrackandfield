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

    def __init__(self, count=200, gens=60):
        self.per_gen = count
        self.gens = gens
        self.chrom_len = 5
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
        chrom_len = len(self.parents[0])
        tot_weight = 0
        weights = []
        temp_parents = []

        for i in range(len(self.parents)):
            tot_weight += self.parents[i][-1]
        
        for i in range(len(self.parents)):
            weights.append(self.parents[i][-1]/tot_weight)

        for i in range(length):

            weight_one = random.uniform(0.9, 1.1)
            weight_two = random.uniform(0.9, 1.1)
            index = np.random.choice([i for i in range(len(weights))], 2, p = weights, replace=False)
            parent_one, parent_two = self.parents[index[0]], self.parents[index[1]]


            child_one = [parent_one[j]*weight_one + parent_two[j]*(1-weight_one) for j in range(chrom_len)]
            child_two = [parent_one[j]*weight_two + parent_two[j]*(1-weight_two) for j in range(chrom_len)]
            temp_parents.append(child_one)
            temp_parents.append(child_two)
        
        self.parents = temp_parents

        return

    #does not use mutation_rate
    def scalar_mutate(self):
        chrom_len = len(self.mutation_only[0])

        for i, chrom in enumerate(self.mutation_only):
            mutant = [self.mutation_only[i][j] * (random.random() + .5) for j in range(chrom_len)]
            self.mutation_only[i] = mutant
    
        return

    def run_chrom(self, chrom, lane):
        node = GazeboTools(lane)
        parameters = {
          "robot_speed": chrom[0],
          "robot_time": chrom[1]
        }
        reward = node.run(parameters)
        return reward, lane

    def run_sim_chrom(self, chrom, lane):
        node = GazeboTools(lane)
        ideal_speed = 10
        ideal_time = 7
        parameters = {
          "robot_speed": chrom[0],
          "robot_time": chrom[1]
        }
        speed_reward = 1 - min((1/8 * abs(chrom[0] - ideal_speed)), 1)
        time_reward = 1 - min((1/8 * abs(chrom[1] - ideal_time)), 1)
        reward = speed_reward + time_reward
        return reward, lane

    def test_gen(self):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for j in range(self.per_gen // 10):
                futures = []
                for i in range(10):
                    futures.append(executor.submit(self.run_sim_chrom, chrom=self.chroms[j*10 + i], lane=i))
                for future in concurrent.futures.as_completed(futures):
                    reward, lane = future.result()
                    self.chroms[j*10 + lane][-1] = reward
                    #print("lane ", lane, "got", reward, "points")

    def sort_and_process_chroms(self):
        self.chroms.sort(key = lambda reward: reward[-1])
       
        # for i, chrom in enumerate(self.chroms):
        #     if i < self.per_gen * .25:
        #         self.mutation_only.append(chrom)
        #     elif i >= self.per_gen * .75:
        #         self.keep.append(chrom)
        #     else:
        #         self.parents.append(chrom)

        self.parents = self.chroms
        
        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("Keep before")
        # for chrom in self.keep:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .25, avg_time/self.per_gen * .25, avg_reward/self.per_gen * .25)

        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("mutate before")
        # for chrom in self.mutation_only:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .25, avg_time/self.per_gen * .25, avg_reward/self.per_gen * .25)

        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("parents before")
        # for chrom in self.parents:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .5, avg_time/self.per_gen * .5, avg_reward/self.per_gen * .5)
        

        self.linear_crossover()
        #self.scalar_mutate()


        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("Keep after")
        # for chrom in self.keep:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .25, avg_time/self.per_gen * .25, avg_reward/self.per_gen * .25)

        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("mutate after")
        # for chrom in self.mutation_only:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .25, avg_time/self.per_gen * .25, avg_reward/self.per_gen * .25)

        # avg_speed, avg_time, avg_reward = 0, 0, 0
        # print("parents after")
        # for chrom in self.parents:
        #     avg_speed += chrom[0]
        #     avg_time += chrom[1]
        #     avg_reward += chrom[-1]
            
        # print(avg_speed/self.per_gen * .5, avg_time/self.per_gen * .5, avg_reward/self.per_gen * .5)

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
            
            print(avg_speed/self.per_gen, avg_time/self.per_gen, avg_reward/self.per_gen)
            f = open('record.txt','a')
            f.write("speed: " + str(avg_speed/self.per_gen) + " time: " + str(avg_time/self.per_gen) + ' reward: ' + str(avg_reward/self.per_gen) + '\n')
            f.close()




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
