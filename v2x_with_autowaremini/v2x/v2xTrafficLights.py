#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import json
from localization.SimulationToUTMTransformer import SimulationToUTMTransformer
from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaTrafficLightStatusList

class CarlaTrafficLightJSONPublisher:
    def __init__(self):
        # Fetch parameters
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Publishers
        self.json_pub = rospy.Publisher('/carla/i2v_message', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/v2x/traffic_light_markers', MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/carla/traffic_lights/info', CarlaTrafficLightInfoList, self.tfl_info_callback)
        rospy.Subscriber('/carla/traffic_lights/status', CarlaTrafficLightStatusList, self.tfl_status_callback, queue_size=1, tcp_nodelay=True)

        # Data holders
        self.traffic_light_info = {}
        self.traffic_light_status = {}

        # Coordinate transformer from simulation coordinates to UTM
        self.sim2utm_transformer = SimulationToUTMTransformer(
            use_custom_origin=use_custom_origin,
            origin_lat=utm_origin_lat,
            origin_lon=utm_origin_lon
        )

        # Timer to periodically publish markers
        #rospy.Timer(rospy.Duration(1.0), self.vizualization_callback)

    def tfl_info_callback(self, msg):
        """
        Callback for traffic light info topic.
        """
        for light in msg.traffic_lights:
            pose = self.sim2utm_transformer.transform_pose(light.transform)

            # Store traffic light info
            self.traffic_light_info[light.id] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "status": "UNKNOWN"
            }

            rospy.loginfo(f"Processing traffic light: {light.id}")

    def tfl_status_callback(self, msg):
        """
        Callback for traffic light status topic.
        """

        for light in msg.traffic_lights:
            if light.id not in self.traffic_light_info:
                rospy.logwarn(f"Traffic light ID {light.id} not found in traffic_light_info. Adding placeholder entry.")
                #self.traffic_light_info[light.id] = {"x": 0.0, "y": 0.0, "status": "UNKNOWN"}

                # Update the status field in the traffic light info
            self.traffic_light_info[light.id]['status'] = light.state

        #print(self.traffic_light_info)
        rospy.loginfo("Received traffic light status update.")
        self.vizualization_callback()
        self.publish_traffic_lights()

    def publish_traffic_lights(self):

        if not self.traffic_light_info:
            rospy.logwarn("No traffic light info available to visualize.")
            return

        #print(self.traffic_light_info.items())
        for light_id, coords in self.traffic_light_info.items():
            if light_id is 43:

                # Construct JSON message
                json_message = {
                    "Header": {
                        "messageType": "DENM",
                        "stationID": f"traffic_light_{light_id}",
                        "timestamp": str(rospy.Time.now().to_sec())
                    },

                    "managementContainer": {
                        "detectionTime": str(rospy.Time.now().to_sec()),
                    },

                    "situationContainer": {
                        "eventType": "badWeather_alert",
                        "eventSeverity": "danger",
                        "trafficLightState": coords["status"]  # red, yellow, green
                    },

                    "locationContainer": {
                        "eventPosition": {
                            "x": coords["x"],
                            "y": coords["y"]
                        },
                        "referencePosition": {
                            "x": coords["x"],
                            "y": coords["y"]
                        }
                    },

                    "aLaCarteContainer": {
                        "hazardDetails": {
                            "hazardType": "trafficSignal",
                            "conditionType": "heavyRain",
                            "intensity": "low"
                        }
                    }
                }


                # Publish JSON message
                self.json_pub.publish(json.dumps(json_message))
                #rospy.loginfo(f"Published traffic light I2V message: {json.dumps(json_message)}")

    def vizualization_callback(self):
        """
        Periodic callback to publish markers for visualization.
        """
        if not self.traffic_light_info:
            rospy.logwarn("No traffic light info available to visualize.")
            return


        markers = MarkerArray()
        for light_id, coords in self.traffic_light_info.items():

            # Map the state to marker colors
            if coords["status"] == 0:
                color = [1.0, 0.0, 0.0]  # Red
            elif coords["status"] == 1:
                color = [1.0, 1.0, 0.0]  # Yellow
            elif coords["status"] == 2:
                color = [0.0, 1.0, 0.0]  # Green
            elif coords["status"] == "UNKNOWN":
                color = [0.5, 0.5, 0.5]  # Grey for off
            else:
                rospy.logwarn(f"Unrecognized traffic light state for ID {light_id}")
                color = [1.0, 1.0, 1.0]  # Default to white for unknown state

            # Create a marker for the traffic light
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "traffic_lights"
            marker.id = light_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = coords["x"]
            marker.pose.position.y = coords["y"]
            marker.pose.position.z = 35.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 0.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 2.0  # Adjust height for better visibility
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            markers.markers.append(marker)

        # Publish all markers
        self.marker_pub.publish(markers)
        rospy.loginfo("Published traffic light markers.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('trafficlight_v2x')
    node = CarlaTrafficLightJSONPublisher()
    node.run()
