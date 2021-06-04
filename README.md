# S.T.R.I.K.E.R 3,000,000 + 1
Final Project for CMSC 20900


## Project Description

* Goal: For this project we aimed to build a robot that could improve it's bowling in our simulated lane through the use of a genetic algorithm. We were able to build a system for training robots to improve their score in bowling by combining our main components into one system. We developed an implementation of bowling in the Gazebo world for which we parameterized certain variables we wished to optimize with our algorithm. The system is built for testing multiple chromosomes at once so we are able to build a genetic algorithm that takes advantage of this parallelization to produce a satisfactory result.
  * Bowling for Robots: The robots bowl in custom built lanes with 10 pins and a ball in front of them. They will push the ball given parameters set by the genetic algorithm. The pins will be measured to see how many were knocked down and this score will be reported as a reward for that set of parameters. (picture of lanes)
  * Parallelizing Training: We gave each lane and robot their own number for gazebo models so we set them independently. We used python’s thread functionality to run the robots in their own lane and record this score. We trained our generations 10 robots at a time. (gif of restart)
  * Training with our Genetic Algorithm: Because we faced issues with Gazebo and the speed of training, we settled on simulating training for most of the process and only brought in the real robots when we approached a solution. This saved time and prevented Gazebo oddities from interfering with the training. (gif of robot bowling poorly then robot bowling better)


## System Architecture

  ### Gazebo World and Parallelization
  
* Creating Gazebo Models: In the models folder, we directly created the XML files for the models we were using (pins, ball, and walls). One interesting aspect is the bowling_pin/model.sdf, where we repositioned the center of mass, and modified the moments of inertia. We tuned these parameters a bit to make knocking down pins feasible in Gazebo.
* Generalizing the Gazebo world: In the script generate_world.py, we wrote a script that can create a gazebo world with any number of lanes and a set number of pins (10) per lane. In the script generate_launch.py, we wrote a script to launch the gazebo world for any given number of lanes, which also creates that number of robots for each lane. After much troubleshooting in Gazebo, we determined that 10 lanes had the best real time factor in Gazebo fluctuating between 0.2 and 0.6. 100 lanes had a real time factor of 0.04.
 * Resetting the Gazebo World and Updating Reward: In the reset_world.py file, we implement the model_states callback which receives the position and orientation of the pins, and generates a reward based on how many are knocked over. We also implemented the reset_world function, which returns everything to their original positions. In the run function, we call the striker.py script, which, given a robot and its lane, gives it a velocity and allows it to bowl. 
    
 
  ### Genetic Algorithm
 Everything here occurs within genetic.py, most of which have their own dedicated functions.
 * Initial Selection and Gen. Size: These are decided in the initialize function. The goal was to be able to run multiple simulations in a row, so they are the default values of per_gen and gen_size in init. We decided on 60 generations of 200 robots apiece since multiples of generation size had a larger effect on the convergence than multiples of generations. We also kept generations relatively low since we can only test 10 robots at a time and wanted to avoid each attempt taking 5 or 6 hours before it exhibited one of the rarer errors. 
 * Testing with Real or Simulated Rewards: We used simulated rewards in the most part as found in run_sim_chrom. Our reward functions for ideal speed and ideal time were designed such that overshooting is just as penalized as undershooting. With this we were able to confirm that our algorithm did improve the parameters we sought  to optimize. Run_chrom was used when we wanted to have the robots actually bowl in the simulation, to ensure that the parameter ideals we chose were reasonable and successful.
 * Keep: Initially our 25% highest performing chromosomes were kept for the next generation without any modifications. Our final iteration would not have any algorithms kept, they would all be available for crossover.
 * Crossover: We tried an initial version of crossover where we selected the middle 50% chromosomes to be paired with each other with equal probability. This worked however it converged slowly, our final iteration made everybody available for crossover and weighed their likelihood of getting chosen to crossover and contribute to the next generation equal to their reward/total reward  for that generation. This meant more fit parents could be chosen more than once.
 * Mutation: Initially we had the poorest performing 25% of our chromosomes be randomly mutated to introduce new genes into the pool and hopefully stumble across a more fit chromosome. Our final algorithm dropped mutation entirely in favor of the more comprehensive crossover described above.
 * Iterating through the Algorithm: Over time we experimented with multiple generation sizes and number of generations, as well as different techniques for mutation and crossover. We determined that generation size was more important than the number of generations, and this understanding coupled with our struggles with Gazebo was the main impetus for introducing a simulated version of the algorithm that we could test and iterate through faster.



  ## Future Work
   * Build a bowling simulator that incorporates more parameters: We could design angle of approach, number of pins in the lane and other variations that could make a more interesting challenge.
   * Build a Genetic Algorithm that converges faster, try new techniques and parameter ranges. We could have experimented ad nauseum with all the possibilities and probabilities involed in choosing a genetic algorithm. Generalized algorithms certainly exist, but they could be specially built as well for particular lanes or number of pins.
   * Build a more complicated bowling lane that tests our algorithm further. We could have built longer lanes, added more pins, added obstacles the robot would have to bowl around. We could have oriented pins in different shapes instead of the typical triangle.
   * Explore other alternatives to simulating bowling than gazebo, as well as exploring different weights of the pins and balls as parameters to be experimented with and optimized for. Since we modified the inertial properties of the objects for this simulation, we could have continued to explore those properties as they pertained to the ball or perhaps even the robot. The bowling lane floor is more slippery than the surrounding floor in real life.

 ## Challenges
   * In the gazebo and parallelization portion of the project, we found that the model objects would not reset properly unless multiple commands to reset were published. The pins would often still have velocity after being reset, such that they would sway or fall over. Dealing with the lag of gazebo once we started parallelizing things, and navigating the gazebo real time factor to find the ideal number of lanes to run was also a challenge. We also tried running Gazebo in headless mode to improve performance, but saw no improvement.   	
   * Gazebo struggles with continuous resets, it introduces errors that our algorithm could not account for. The robot would bowl at different angles even though it was placed directly in front of the ball every time. No matter our algorithm implementation reward would fall to zero because the lanes just get messier and messier. This would prove to be consistently frustrating.
   * People on the team had issues with their NoMachine at various points throughout the final project.
   * Real training takes far too long for our algorithm to converge on something acceptable, this is why simulated training was necessary.
  * Even with a small generation size, it can easily take upwards of an hour before errors occur while using gazebo simulation. This causes development to take a very long time for even simple changes to ensure that they do not cause any errors.
   * We did not expect the sheer size of design choices that were possible when making a genetic algorithm. Everything from generation size and count, to the randomness in mutating and crossover, to the techniques used, to how we measure reward were all up for discussion. Discovering the appropriate degree of randomness and what ranges were acceptable given what values we were toying with took a significant amount of time. By the time we found something that would converge, modifying it further seemed reckless.

## Takeaways
 * Gazebo is a harsh mistress, there will be errors and deviations that will be almost impossible to find unless you run your code for a sufficiently long period of time.
 * Genetic Algorithms are more of an art than a science, their modularity and customizability allows custom solutions for particular workloads, but also introduce sources of randomness that can make converging more difficult if it is not properly tuned. What "properly tuned" means is up for debate, further complicating the issue.
