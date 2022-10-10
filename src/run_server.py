
import rospy
from briit_server import ServerNode
def server_node():
    rospy.init_node("run_server")
    
    serv = server_node.server_node("1234")
    rate = rospy.Rate(15)
    
    while not rospy.is_shutdown():     
        rospy.loginfo_once("Server Program Ready")   #serv.getHeartBeat()
        serv.postImage()
        serv.triggerDrone()
        rate.sleep()
    
if __name__ == "__main__":
    try :
        server_node()
    except rospy.ROSInterruptException :
        rospy.loginfo("Server Node Failed to Start")
        pass