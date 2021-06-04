#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from striker import Striker

import time

class GazeboTools(object):

  def __init__(self, lane_number):
    self.lane = lane_number
    self.initialized = False

    self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1000)
    self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_received)

    y_offset = 2 * self.lane
  
    self.pins = {
          f"lane_{self.lane}_pin1": [3, y_offset, 0],
          f"lane_{self.lane}_pin2": [3.1732, -0.1+y_offset, 0],
          f"lane_{self.lane}_pin3": [3.1732, 0.1+y_offset, 0],
          f"lane_{self.lane}_pin4": [3.346, -0.2+y_offset, 0],
          f"lane_{self.lane}_pin5": [3.346, y_offset, 0],
          f"lane_{self.lane}_pin6": [3.346, 0.2+y_offset, 0],
          f"lane_{self.lane}_pin7": [3.520, -0.3+y_offset, 0],
          f"lane_{self.lane}_pin8": [3.520, -0.1+y_offset, 0],
          f"lane_{self.lane}_pin9": [3.520, 0.1+y_offset, 0],
          f"lane_{self.lane}_pin10": [3.520, 0.3+y_offset, 0],
          f"lane_{self.lane}_ball": [1, 0+y_offset, 0],
          f"lane_{self.lane}_robot": [0, y_offset, 0]
        }
    self.pin_states = {}
    self.reward = 0
    self.reset_clock_running = False
    self.please_reset = False
    self.initialized = True
    self.counter = 0

  def model_states_received(self, data):
    robot_pose = None

    # Only run this every 100 times bc
    # I think it gets kinda slow, and starts
    # reporting a score of 0 too much
    self.counter += 1
    if self.counter % 100 != 0:
      return
    self.counter = 0

    for pin_name, pose in self.pins.items(): #Check each pin to see if it is knocked over
      if f"lane_{self.lane}" not in pin_name:
        continue
      block_idx = data.name.index(pin_name)
      theta = data.pose[block_idx].orientation
      roll, pitch, yaw = euler_from_quaternion([
            theta.x,
            theta.y,
            theta.z,
            theta.w])
      if "robot" in pin_name:
        robot_pose = data.pose[block_idx].position
      elif "pin" in pin_name:
        knocked_down = abs(roll) > 0.4 or abs(pitch) > 0.4
        self.pin_states[pin_name] = knocked_down
    reward = 0
    for pin in self.pin_states: #If it is knocked over add to the reward
      if self.pin_states[pin]:
        reward += 1
    self.reward = reward

  def reset_world(self): #function to put all the pins and robot back into their original positions
    q_list = quaternion_from_euler(0, 0, 0)
    q = Quaternion()
    q.x = q_list[0]
    q.y = q_list[1]
    q.z = q_list[2]
    q.w = q_list[3]

    for pin in self.pins: #publish a model state for each object in Gazebo
      rospy.sleep(0.01)
      p = Pose(position=Point(x=self.pins[pin][0], y=self.pins[pin][1], z=self.pins[pin][2]), orientation=q)
      t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
      model_state = ModelState(model_name=pin, pose=p, twist=t)
      model_state.reference_frame = "world"
      self.set_model_state.publish(model_state)

  def run(self, parameters):
    bot = Striker(self.lane)
    rospy.sleep(1)
    # Extra reset just to make sure the bot is really stopped
    bot.stop()
    self.reset_world()
    start_time = rospy.Time.now().to_sec()
    bot.bowl(parameters["robot_speed"], min(abs(parameters["robot_time"]), 7))
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
      if rospy.Time.now().to_sec() - start_time > 7:
        current_reward = self.reward
        for i in range(5):
            self.reset_world()
            bot.stop()
            rospy.sleep(0.01)
        return current_reward
      r.sleep()
