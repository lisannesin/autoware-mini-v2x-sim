#!/usr/bin/env python3
"""
Subscribe to /detected_objects, filter for cars (or pedestrians), send I2V JSON messages,
and visualize each detected object as a moving cube in RViz.
"""

import rospy
import json
from std_msgs.msg import String
from autoware_msgs.msg import DetectedObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class PedestrianV2XProcessor:

    def __init__(self):
        rospy.loginfo("Initializing Pedestrian V2X Processor")

        # Publishers
        self.json_pub = rospy.Publisher('/carla/v2p_message', String, queue_size=1)
        self.marker_pub = rospy.Publisher('/v2x/pedestrian_markers', MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('/detection/final_objects',DetectedObjectArray,self.detected_objects_callback,queue_size=10)

    def detected_objects_callback(self, msg):
        """
        Process incoming DetectedObjectArray
        publish JSON messages, and visualize each detection.
        """
        marker_array = MarkerArray()

        for obj in msg.objects:
            if obj.label != "pedestrian":
                continue

            x = obj.pose.position.x
            y = obj.pose.position.y

            # --- JSON part ---
            json_message = {
                "Header": {
                    "messageType": "DENM",
                    "stationID": f"{obj.label}_{obj.id}",
                    "timestamp": str(rospy.Time.now().to_sec())
                },
                "managementContainer": {
                    "detectionTime": str(rospy.Time.now().to_sec())
                },
                "situationContainer": {
                    "eventType": "badWeather_alert",
                    "eventSeverity": "danger",
                },
                "locationContainer": {
                    "eventPosition": {
                        "x": x,
                        "y": y
                    },
                    "referencePosition": {
                        "x": x,
                        "y": y
                    }
                },
                "aLaCarteContainer": {
                    "hazardDetails": {
                        "hazardType": "weatherCondition",
                        "conditionType": "heavyRain",
                        "intensity": "low"
                    }
                }
            }

            self.json_pub.publish(json.dumps(json_message))
            rospy.loginfo(f"[P2V] {obj.label.capitalize()} Alert JSON published: ({x}, {y})")

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "simple_marker"
            marker.id = obj.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 35
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.5
            marker.scale.y = 2.5
            marker.scale.z = 2.8
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
            marker.lifetime = rospy.Duration(0.5)

            marker_array.markers.append(marker)

        # publish all markers at once
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pedestrian_v2x')
    node = PedestrianV2XProcessor()
    node.run()
