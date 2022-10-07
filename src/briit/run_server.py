import rospy
from briit import ServerBRIIT, rospy

def server_node():
    rospy.init_node("run_server")
    serv = ServerBRIIT("1234")
    rate = rospy.Rate(15)
    while (True):
        serv.getHeartBeat()
        serv.triggerDrone()
        rate.sleep()

if __name__ == "__main__":
    try :
        server_node()
    except rospy.ROSInterruptException :
        rospy.loginfo("Server Node Failed to Start")
        pass