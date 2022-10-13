
import rospy
from briit_server import server_node
def server_run():
    rospy.init_node("run_server")
    
    s_run = server_node.ServerNode("1234")
    rate = rospy.Rate(15)
    
    while not rospy.is_shutdown():     
        rospy.loginfo("Tes 1")
        if s_run.getHeartBeat():
            s_run.triggerDrone()
        rospy.loginfo_once("Server Program Ready")  
        rospy.loginfo("test")
        rate.sleep()
    
if __name__ == "__main__":
    try :
        server_run()
    except rospy.ROSInterruptException :
        rospy.loginfo("Server Node Failed to Start")
        pass