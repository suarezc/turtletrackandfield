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

    def __init__(self, count=200, gens=100):

        #initialize the chromosomes and the starting range of values
        self.per_gen = count
        self.gens = gens
        self.chrom_len = 5
        self.vel_start_min = 4
        self.vel_start_range = 7
        self.time_range = 6
        self.parents = [] #middle 50

        self.chroms = []
        for i in range(self.per_gen):
            self.chroms.append([random.random()*self.vel_start_range+self.vel_start_min, #velocity
                                random.random()*self.time_range, #time
                                -1]) #reward always last index

        self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=60)
        return


    def linear_crossover(self):

        #randomly distribute parents amd prepare for weighting
        random.shuffle(self.parents)
        length = len(self.parents)//2
        chrom_len = len(self.parents[0])
        tot_weight = 0
        weights = []
        temp_parents = []

        #find total value of rewards
        for i in range(len(self.parents)):
            tot_weight += self.parents[i][-1]
        
        #associate each chromosome with its proportion of the total reward
        for i in range(len(self.parents)):
            weights.append(self.parents[i][-1]/tot_weight)

        #choose parents randomly based on their weighting and then apply our crossover technique
        for i in range(length):

            #weights are kept tight so that there is a limited amount of drift per generation
            weight_one = random.uniform(0.9, 1.1)
            weight_two = random.uniform(0.9, 1.1)
            #parents are chosen here
            index = np.random.choice([i for i in range(len(weights))], 2, p = weights, replace=False)
            parent_one, parent_two = self.parents[index[0]], self.parents[index[1]]

            #Two children are produced from every pair of parents, they will be in the next generation
            #Our weights are applied to every aspect of the chromosome, changing reward does not matter since this will only trigger after they have been tested
            child_one = [parent_one[j]*weight_one + parent_two[j]*(1-weight_one) for j in range(chrom_len)]
            child_two = [parent_one[j]*weight_two + parent_two[j]*(1-weight_two) for j in range(chrom_len)]

            temp_parents.append(child_one)
            temp_parents.append(child_two)
        
        #parents die off, children are the future
        self.parents = temp_parents

        return

    #vestigial code from our mutation experiments
    # def scalar_mutate(self):
    #     chrom_len = len(self.mutation_only[0])

    #     for i, chrom in enumerate(self.mutation_only):
    #         mutant = [self.mutation_only[i][j] * (random.random() + .5) for j in range(chrom_len)]
    #         self.mutation_only[i] = mutant
    
    #     return


    #Our robot bowl function. This plugs into the reset and striker code in order to move the bots in gazebo
    def run_chrom(self, chrom, lane):
        node = GazeboTools(lane)
        parameters = {
          "robot_speed": chrom[0],
          "robot_time": chrom[1]
        }
        reward = node.run(parameters)
        return reward, lane

    #Our simulated function for reward generatoin
    def run_sim_chrom(self, chrom, lane):
        node = GazeboTools(lane)

        #These are parameters we tested and believe will create good bowlers
        ideal_speed = 3.5
        ideal_time = 1.5
        parameters = {
          "robot_speed": chrom[0],
          "robot_time": chrom[1]
        }

        #Reward functions are designed such that overshooting and undershooting are penalized equally. Bigger is not always better here
        speed_reward = 1 - min((1/8 * abs(chrom[0] - ideal_speed)), 1)
        time_reward = 1 - min((1/8 * abs(chrom[1] - ideal_time)), 1)

        #Reward is min 0 and max 2, the reward is the simple addition of the two components
        reward = speed_reward + time_reward
        return reward, lane

    #Test our robots using their own threads
    def test_gen(self):
        #Python built in thread manager
        with concurrent.futures.ThreadPoolExecutor() as executor:
            #Test ten robots at a time
            for j in range(self.per_gen // 10):
                futures = []

                for i in range(10):
                    #if first or last gen, let the robots actually bowl
                    if self.gens == 1 or self.gens == 100:
                        futures.append(executor.submit(self.run_chrom, chrom=self.chroms[j*10 + i], lane=i))

                    #otherwise use our sim function for speedier testing
                    else:
                        futures.append(executor.submit(self.run_sim_chrom, chrom=self.chroms[j*10 + i], lane=i))

                #save reward for processing down the line, reward is always the final index
                for future in concurrent.futures.as_completed(futures):
                    reward, lane = future.result()
                    self.chroms[j*10 + lane][-1] = reward

    #Method where chromosomes were chosen to be kept, mutated or bred. Final iteration has them all being bred
    def sort_and_process_chroms(self):
       
        #array for parents
        self.parents = self.chroms
        #used to use mutation method as weell but simple crossover proved to be superior
        self.linear_crossover()

        #new chroms are assigned
        self.chroms = self.parents

        return


    #Statistics are calculated and printed here on a per generation basis as well as the running of the generations
    def run(self):
        while not rospy.is_shutdown() and self.gens != 0:            
            print("running " + str(self.gens))
            t_start = datetime.now()

            #call the function that will run real or simulated bowling
            self.test_gen()

            t_end = datetime.now()
            #Get generation test timing.
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
            
            #print average stats for the generation for sanity checking
            print(avg_speed/self.per_gen, avg_time/self.per_gen, avg_reward/self.per_gen)

            self.sort_and_process_chroms()
            self.parents = []


            #reset reward
            for chrom in self.chroms:
                chrom[-1] = -1

            #generation is complete, decrement counter
            self.gens -= 1

#Main method, declare and run nodes here
if __name__ == "__main__":
    rospy.init_node("genetic")
    gen = Genetic()
    gen.run()
