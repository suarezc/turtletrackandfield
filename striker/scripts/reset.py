import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GazeboTools(object):

  def __init__(self):

    rospy.init_node("reset_world")

    self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    rospy.Subscriber("gazebo/model_states", ModelStates, self.model_states_received)

    self.pins = {
          "pin1": [3, 0, 0],
          "pin2": [3.1732, -0.1, 0],
          "pin3": [3.1732, 0.1, 0],
          "pin4": [3.346, -0.2, 0],
          "pin5": [3.346, 0, 0],
          "pin6": [3.346, 0.2, 0],
          "pin7": [3.520, -0.3, 0],
          "pin8": [3.520, -0.1, 0],
          "pin9": [3.520, 0.1, 0],
          "pin10": [3.520, 0.3, 0],
          "ball": [1, 0, 0],
        #  "TODO figure out robot name": [0, 0, 0]
        }
    self.current_numbered_blocks_locations = None
    self.reward = 0 

  def model_states_received(self, data):
    # TODO, compute score, check if it should be reset
    #print(data)
    if (self.current_numbered_blocks_locations == None):
      self.current_numbered_blocks_locations = {}
    for pin_name,pose in self.pins.items():
      block_idx = data.name.index(pin_name)
      self.current_numbered_blocks_locations[pin_name] = data.pose[block_idx].orientation
      theta = self.current_numbered_blocks_locations[pin_name]
      yaw = (euler_from_quaternion([
            theta.x,
            theta.y,
            theta.z,
            theta.w])
            [2])
      print(yaw)
    print("____________")
    rospy.sleep(2)
      #if z > 0.3:
        #self.reward += 1
    




  def reset_world(self):
    q_list = quaternion_from_euler(0, 0, 0)
    q = Quaternion
    q.x = q_list[0]
    q.y = q_list[1]
    q.z = q_list[2]
    q.w = q_list[3]

    for pin in self.pins:
      p = Pose(position=Point(x=self.pins[pin][0], y=self.pins[pin][1], z=self.pins[pin][2]), orientation=q)

      t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
      model_state = ModelState(model_name=pin, pose=p, twist=t)
      model_state.reference_frame = "world"
      self.set_model_state.publish(model_state)

  def actually_reset_world(self):
    """
    idk why but this works and the other doesn't
    """

    for i in range(20):
      self.reset_world()

  def run(self):
          rospy.spin()

if __name__=="__main__":

    node = GazeboTools()
    node.run()