#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
import requests
from briit.msg import GPSCoordinate

url = "http://172.17.0.76:5000/v1"
health_endpoint = "/health" # METHODE GET
image_flag_endpoint = "/flag/image-to-json" # METHOD POST
house_flag_endpoint = "/house/image-to-json" # METHOD POST
drone_endpoint = "/drone" # METHODE GET (TRIGGER)

class ServerBRIIT():
    def __init__(self,device_id):
        self.device_id = device_id
        self.coordinate = GPSCoordinate()
        self.error = False
        self.msg = "No Message"
        self.server_status = rospy.Publisher("/briit/server_status",String,queue_size=10)
        self.position_user = rospy.Publisher("/briit/position_user",GPSCoordinate,queue_size=10)
    
    def getHeartBeat(self):
        try:
            connection = requests.get(url+health_endpoint)
            conn_data_hb = connection.json()
            if conn_data_hb == 'OK':
                self.status = True
                self.server_status.publish(conn_data_hub)
            else:
                self.status = False
                self.server_status.publish("Not OK")
            return self.status
        except Exception as e:
            self.msg = str(e)
            self.error = True
    
    def triggerDrone(self):
        try:
            connection = requests.get(url+drone_endpoint,data={'deviceid':self.device_id})
            constatus = connection.status_code
            if connstatus == 200:
                if connection.text.find("is empty") == -1:
                    self.coordinate.lat = connection.json()['data']['lat']
                    self.coordinate.alt = connection.json()['data']['alt']
                    self.coordinate.long = connection.json()['data']['long']
                else:
                    ros.loginfo("No Data!")
            else:
                ros.loginfo("No WiFi!")
        except Exception as e:
            self.msg = str(e)
            self.error = True
    
    def postImage(self):
        pass
