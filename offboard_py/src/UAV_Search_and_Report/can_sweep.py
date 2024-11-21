import rospy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import time

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"Received: {data.data}")
    if data.data:

        # pull one relay low to extend the lin-ac
        GPIO.output(Relay_Pin1,GPIO.LOW)
        rospy.loginfo(rospy.get_caller_id() + f"extending linear actuator...")
        
        # wait for full extension
        time.sleep(8)

        # switch both relays to retract the lin-ac
        GPIO.output(Relay_Pin1, GPIO.HIGH)
        GPIO.output(Relay_Pin2,GPIO.LOW)
        rospy.loginfo(rospy.get_caller_id() + f"retracting linear actuator...")
        
        # wait for full retraction
        time.sleep (8)

        # update wiper status
        rospy.loginfo(rospy.get_caller_id() + f"equipment replenished")




def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("release", Bool, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    # set GPIO pins to hold both relays high (motor off)
    Relay_Pin1 = 18 # GPIO onJetson
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(Relay_Pin1,GPIO.OUT)
    GPIO.outpuot(Relay_Pin1, GPIO.HIGH)
    
    Relay_Pin2 = 22 # GPIO onJetson
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(Relay_Pin2,GPIO.OUT)
    GPIO.outpuot(Relay_Pin2, GPIO.HIGH)
    
    listener()