#!/usr/bin/env python

import rospy
from rb1_localization.srv import POI, POIRequest, POIResponse

class RecordPOIServer():
    SERVICE_NAME = "record_poi_server"

    def __server_request_callback(self, request: POIRequest):
        response = POIResponse()
        response.success = True
        return response

    def __init__(self):
        self._service = rospy.Service(RecordPOIServer.SERVICE_NAME, POI , self.__server_request_callback)
        rospy.loginfo(f"Service {RecordPOIServer.SERVICE_NAME} Ready")
        self._pois = []
        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

            
if __name__ == '__main__':
    rospy.init_node(RecordPOIServer.SERVICE_NAME, anonymous=True)
    recordPOIServer = RecordPOIServer()
    rospy.spin()
