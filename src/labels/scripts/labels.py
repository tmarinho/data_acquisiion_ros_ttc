#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import numpy.linalg
import copy as cp

MULTIRATE = 6
VICON_RATE = 100.0
VICON_SAMPLE_TIME = 1.0/VICON_RATE
class LabelGenerator:
  """Computes ground truth (labels) based on Vicon stream.
     Reads 2 consecutive Vicon measurements from robots
     and computes velocity by numerical differentiation.
     Direct computation os line-of-sight (LOS), time-to-
     -collision (TTC) and LOS rate."""

  def __init__(self):
    self.x = 0
    self.buffer = []

  def sg_filter(self, x, m, k=0):
    """
    x = Vector of sample times
    m = Order of the smoothing polynomial
    k = Which derivative
    """
    mid = len(x) / 2
    a = x - x[mid]
    def expa(x): return map(lambda i: i**x, a)
    A = np.r_[map(expa, range(0, m+1))].transpose()
    Ai = np.linalg.pinv(A)

    return Ai[k]

  def smooth(self, x, y, order=2, deriv=1):

    if deriv > order:
      raise Exception, "deriv must be <= order"
  
    f = self.sg_filter(x, order, deriv)
    result = np.dot(f, y)

    return result

  def line_of_sight(self, x1, y1, x2, y2):
    xd = x2 - x1
    yd = y2 - y1
    return math.atan2(xd, yd)

  def callback(self, msg):
    self.x = msg.pose.position.x
    self.y = msg.pose.position.y
    self.buffer.append(self.x)


  def ground_truth_talker(self):

    rospy.init_node('labels')
    rospy.Subscriber('/vicon/jim', PoseStamped, self.callback)
    publisher_los = rospy.Publisher('los', Float32, queue_size = 10)
    # publisher_los_rate = rospy('los_rate')
    # publisher_ttc = rospy('ttc')
    rate = rospy.Rate(VICON_RATE) #Hz
    multirate_counter = 0
    while not rospy.is_shutdown():
      
      # Publishes at VICON_RATE/MULTIRATE Hz
      if multirate_counter == MULTIRATE:
        position = cp.copy(self.buffer)
        m = len(position)
        t = np.array([x*VICON_SAMPLE_TIME for x in range(0,m)])
        velocity_x = self.smooth(t, position, 2 , deriv=1)
        publisher_los.publish(velocity_x)
        rospy.loginfo('Velocity:{}'.format(velocity_x))
        self.buffer = []
        multirate_counter = 0
      multirate_counter = multirate_counter + 1
      rate.sleep()


if __name__ == '__main__':
  try:
    obj = LabelGenerator()
    obj.ground_truth_talker()
  except rospy.ROSInterruptException:
    pass


    # t, pos = load(data)
    # params = (t, pos, size, order)

    # plots = [
    #     ["Position",     pos],
    #     ["Velocity",     smooth(*params, deriv=1)],