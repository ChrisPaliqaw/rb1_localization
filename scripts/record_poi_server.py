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
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

def poseTupleTostring(pose_tuple):
    title = pose_tuple[0]
    pose = pose_tuple[1]
    return f"""
{title}:
  position:
    x: {pose.position.x}
    y: {pose.position.y}
    z: {pose.position.z}
  orientation:
    x: {pose.orientation.x}
    y: {pose.orientation.y}
    z: {pose.orientation.z}
    w: {pose.orientation.w}
           """

class RecordPOIServer():
    SERVICE_NAME = "save_poi"
    WAIT_HZ = 1
    END_MESSAGE = "end"
    OUT_FILE_NAME = "poi.yaml"

    def __writeFile(self):
        with open(RecordPOIServer.OUT_FILE_NAME, 'w') as f:
            for pose_tuple in self._pois:
                f.write(poseTupleTostring(pose_tuple))

    def __serverRequestCallback(self, request: POIRequest):
        success = True
        if request.label == RecordPOIServer.END_MESSAGE:
            if len(self._pois) > 0:
                self.__writeFile()
                rospy.loginfo(f"Wrote {RecordPOIServer.OUT_FILE_NAME}")
                self._pois.clear()
            else:
                success = False
        else:
            while self._last_amcl_pose is None:
                rospy.loginfo("Waiting for /amcl_pose data...")
                rospy.sleep(RecordPOIServer.WAIT_HZ)
            self._pois.append((request.label, self._last_amcl_pose))
            rospy.loginfo(f"POI list length = {len(self._pois)}")
            rospy.loginfo(f"{poseTupleTostring(self._pois[-1])}")
        response = POIResponse()
        response.success = success
        return response

    def __amclPoseCallback(self, poseWithCovarianceStamped: PoseWithCovarianceStamped):
        self._last_amcl_pose = poseWithCovarianceStamped.pose.pose
        # rospy.loginfo(self._last_amcl_pose)

    def __init__(self):
        self._service = rospy.Service(RecordPOIServer.SERVICE_NAME, POI , self.__serverRequestCallback)
        self._amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.__amclPoseCallback)
        rospy.loginfo(f"Service {RecordPOIServer.SERVICE_NAME} Ready")
        self._pois = []
        self._last_amcl_pose = None
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

            
if __name__ == '__main__':
    rospy.init_node("record_poi_server", anonymous=True)
    recordPOIServer = RecordPOIServer()
    rospy.spin()
