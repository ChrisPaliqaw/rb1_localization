#!/usr/bin/env python

"""
POI.serv:

string label
---
bool success      # Did it achieve it?
"""

"""
geometry_msgs/PoseWithCovarianceStamped:

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
"""

import rospy
from rb1_localization.srv import POI, POIRequest, POIResponse
from geometry_msgs.msg import PoseWithCovarianceStamped

class RecordPOIServer():
    SERVICE_NAME = "record_poi_server"
    WAIT_HZ = 1

    def __server_request_callback(self, request: POIRequest):
        response = POIResponse()
        response.success = True
        return response

    def __amcl_pose_callback(self, poseWithCovarianceStamped: PoseWithCovarianceStamped):
        self._last_amcl_pose = poseWithCovarianceStamped.pose.pose
        rospy.loginfo(self._last_amcl_pose)

    def __init__(self):
        self._service = rospy.Service(RecordPOIServer.SERVICE_NAME, POI , self.__server_request_callback)
        self._amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.__amcl_pose_callback)
        rospy.loginfo(f"Service {RecordPOIServer.SERVICE_NAME} Ready")
        self._pois = []
        self._last_amcl_pose = None
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

            
if __name__ == '__main__':
    rospy.init_node(RecordPOIServer.SERVICE_NAME, anonymous=True)
    recordPOIServer = RecordPOIServer()
    rospy.spin()
