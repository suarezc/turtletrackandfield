# S.T.R.I.K.E.R 3,000,000 + 1
Final Project for CMSC 20900


## Project Description

* Goal: For this project we aimed to build a robot that could improve it's bowling in our simulated lane through the use of a genetic algorithm. We were able to build a system for training robots to improve their score in bowling by combining our main components into one system. We developed an implementation of bowling in the Gazebo world for which we parameterized certain variables we wished to optimize with our algorithm. The system is built for testing multiple chromosomes at once so we are able to build a genetic algorithm that takes advantage of this parrallelization to produce a satisfactory result.
  * Bowling for Robots: (how the robot bowls and how the lanes were designed) (picture of lanes)
  * Parrellizing Training: (how we got many robots to bowl at once and how the reset mechanism works) (gif of restart)
  * Training with our Genetic Algorithm: (how we use sim training in conjunction with real training to produce our bowling bot) (gif of robot bowling poorly then    robot bowling better)


## System Architecture
  * Genetic Algorithm
    * Initial Selection and Gen. Size:
    * Testing with Real or Simulated Rewards:
    * Crossover:
    * Mutation:
    * Iterating through the Algorithm:



## Challenges, Future Work, and Takeaways
  ## Future Work
   * Build a bowling simulator that incorporates more parameters:
   * Build a Genetic Algorithm that converges faster, try new techniques and parameter ranges:
   * Build a more complicated bowling lane that tests our algorithm further:

 ## Challenges
   * Gazebo struggles with continuous resets, it introduces errors that our algorithm could not account for. The robot would bowl at different angles even though it was placed directly in front of the ball every time. No matter our algorithm implementation reward would fall to zero because the lanes just get messier and messier. This would prove to be consistently frustrating.
   * People on the team had issues with their NoMachine at various points throughout the final project.
   * Real training takes far too long for our algorithm to converge on something acceptable, this is why simulated training was necessary.
   * We did not expect the sheer size of design choices that were possible when making a genetic algorithm. Everything from generation size and count, to the randomness in mutating and crossover, to the techniques used, to how we measure reward were all up for discussion. Discovering the appropriate degree of randomness and what ranges were acceptable given what values we were toying with took a significant amount of time. By the time we found something that would converge, modifying it further seemed reckless.

## Takeaways
 * Gazebo is a harsh mistress, there will be errors and deviations that will be almost impossible to find unless you run your code for a sufficiently long period of time.
 * Genetic Algorithms are more of an art than a science, their modularity and customizability allows custom solutions for particular workloads, but also introduce sources of randomness that can make converging more difficult if it is not properly tuned. What "properly tuned" means is up for debate, further complicating the issue.
 * 
