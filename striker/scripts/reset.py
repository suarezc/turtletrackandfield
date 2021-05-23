#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GazeboTools(object):

  def __init__(self, lane_number):
    self.lane = lane_number
    self.initialized = False
    rospy.init_node("reset_world")

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
    self.current_numbered_blocks_locations = None
    self.reward = 0
    self.reset_clock_running = False
    self.please_reset = False
    rospy.sleep(1)
  
    self.initialized = True

  def model_states_received(self, data):
    print("got model states")
    #print(rospy.Time.now().to_sec())
    for pin_name, pose in self.pins.items():
      if not "pin" in pin_name:
        continue
      block_idx = data.name.index(pin_name)
      theta = data.pose[block_idx].orientation
      roll, pitch, yaw = euler_from_quaternion([
            theta.x,
            theta.y,
            theta.z,
            theta.w])
      knocked_down = abs(roll) > 0.4 or abs(pitch) > 0.4
      self.pin_states[pin_name] = knocked_down
    reward = 0
    for pin in self.pin_states:
      if self.pin_states[pin]:
        reward += 1
    self.reward = reward
    #print(self.reward, self.reset_clock_running, rospy.Time.now().to_sec())
    if self.reward > 0:
      if self.reset_clock_running:
        t1 = rospy.Time.now().to_sec()
        if t1 - self.t0 > 5:
          self.reset_clock_running = False
          self.reward = 0
          self.please_reset = True
          # self.reset_world()
          # self.model_state_sub.unregister()
          # rospy.sleep(1)
          # self.model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, self.model_states_received)

      else:
        print("reset clock started")
        self.t0 = rospy.Time.now().to_sec()
        self.reset_clock_running = True


  def reset_world(self):
    q_list = quaternion_from_euler(0, 0, 0)
    q = Quaternion
    q.x = q_list[0]
    q.y = q_list[1]
    q.z = q_list[2]
    q.w = q_list[3]

    for pin in self.pins:
      print("resetting pin")
      rospy.sleep(0.01)
      p = Pose(position=Point(x=self.pins[pin][0], y=self.pins[pin][1], z=self.pins[pin][2]), orientation=q)

      t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
      model_state = ModelState(model_name=pin, pose=p, twist=t)
      model_state.reference_frame = "world"
      self.set_model_state.publish(model_state)

  def run(self, ball_start_state=None):
    print("starting run")
    rospy.sleep(1)
    if ball_start_state is not None and self.initialized:
        self.set_model_state.publish(ball_start_state)
        print("velocity set")
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
      print('still in run')
      if self.please_reset:
        print("time to reset")
        self.please_reset = False
        self.reset_world()
        print("done resetting! reward is " + str(self.reward))
        return self.reward
      r.sleep()
      #print("hi")

if __name__=="__main__":
    print("started main")
    node = GazeboTools(1)
    node.run()
