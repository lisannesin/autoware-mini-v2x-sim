#!/usr/bin/env python3

import rospy
import json
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped


class CarlaJSONProcessor:
    def __init__(self):

        self.current_x = None
        self.current_y = None
        self.current_speed = 0.0
        self.current_speedy = 0.0


        self.MAX_DISTANCE = 50.0
        self.TTC_THRESHOLD = 50.0

        #Subscribers
        rospy.Subscriber('/localization/current_pose',PoseStamped,self.pose_callback,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/carla/i2v_message', String, self.process_v2x_message)
        rospy.Subscriber('/carla/v2p_message', String, self.process_v2x_message)
        rospy.Subscriber('/carla/i2v_message', String, self.process_v2x_message)


    def pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def current_velocity_callback(self, msg: TwistStamped):
        self.current_speed = msg.twist.linear.x

    def execute_emergency_braking(self):
        rospy.logerr("EMERGENCY: Applying full brakes!")

    def execute_preemptive_slowdown(self):
        rospy.logwarn("PRECAUTION: Reducing speed to avoid collision.")

    def process_traffic_signal(self, traffic_light_state: str):
        if traffic_light_state == 0:
            rospy.loginfo("Red light detected. Preparing to stop.")
        elif traffic_light_state == 2:
            rospy.loginfo("Green light detected. Proceeding safely.")
        elif traffic_light_state == 1:
            rospy.loginfo("Yellow light detected. Reducing speed.")

    def process_danger_event(self, message: dict):
        event_type = message["situationContainer"]["eventType"]
        rospy.loginfo(f" DANGER: {event_type} detected! Taking immediate action!")

        if event_type == "dangerous_vehicle" or event_type == "hidden_pedestrian":
            self.execute_emergency_braking()

        else:
            station_id = message["Header"]["stationID"]

            if station_id.startswith("traffic_light"):
                self.process_traffic_signal(message["situationContainer"]["trafficLightState"])

            elif station_id.startswith("pedestrian"):
                self.execute_emergency_braking()

            elif station_id.startswith("car"):
                self.execute_emergency_braking()


    def process_v2x_message(self, data: String):
        try:
            message = json.loads(data.data)

            # Extract vehicle current position and velocity
            if self.current_x is None or self.current_y is None:
                rospy.loginfo("Waiting for current_pose")
                return

            event_x = message["locationContainer"]["eventPosition"]["x"]
            event_y = message["locationContainer"]["eventPosition"]["y"]

            # Step 1
            dx = event_x - self.current_x
            dy = event_y - self.current_y
            distance = math.hypot(dx, dy)
            if distance > self.MAX_DISTANCE:
                rospy.loginfo(f"Distance = {distance:.1f} discarding message.")
                return
            else:
                rospy.loginfo(f"Distance = {distance:.1f} keep processing.")

            # Step 2
            #vx = 10
            vx = self.current_speed
            vy = self.current_speedy
            speed_mag = math.hypot(vx, vy)

            if speed_mag < 0.01:
                rospy.loginfo("Vehicle speed 0 not moving")
                return

            #closing_speed = -((vx * dx + vy * dy))
            ttc = distance / vx

            if ttc > self.TTC_THRESHOLD:
                rospy.loginfo(f"TTC = {ttc:.2f} discarding.")
                return
            else:
                rospy.loginfo(f"TTC = {ttc:.2f} keep processing.")

            # Step 3
            severity = message["situationContainer"]["eventSeverity"].lower()
            if severity == "danger":
                rospy.loginfo("Danger")
                self.process_danger_event(message)
                return

            elif severity == "warning":
                rospy.loginfo("Warning")
                self.execute_preemptive_slowdown()
                return

            else:
                rospy.loginfo(f"Unknown")
                return

        except json.JSONDecodeError:
            rospy.logerr("Failed to decode JSON message")
        except KeyError as e:
            rospy.logwarn(f"ERROR: Missing key in incoming V2X message: {e}")
            return

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('v2xPros')
    node = CarlaJSONProcessor()
    node.run()
