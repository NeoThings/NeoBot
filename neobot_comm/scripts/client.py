#!/usr/bin/env python3

import rospy
import httpclient as http_client
import mqttclient as mqtt_client

class NetworkClient():
  def __init__(self, name):
    self.name = name
    self.rospy = rospy
    self.rospy.init_node(self.name, anonymous = True)
    self.rospy.loginfo("[%s] Initiated ", self.name)
    self.initParameters()
    self.initVariables()
    #self.pub_path = self.rospy.Publisher(self.path_topic, Path, queue_size = 1, latch = True)
    self.rospy.spin()

if __name__ == '__main__':
    try:
        network_client = NetworkClient('network_client')
    except rospy.ROSInterruptException:
        pass