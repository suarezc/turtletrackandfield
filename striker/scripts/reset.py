import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class GazeboTools:

  def __init__(self):
    rospy.init_node("reset_world")

    self.set_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

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
        #  "TODO figure out robot name": [0, 0, 0],
        }

  def model_state_received(self, data):
    # TODO, compute score, check if it should be reset
    pass

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
