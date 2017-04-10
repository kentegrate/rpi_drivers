from std_msgs.msg import Float32
import rospy
import time
def main():
    rospy.init_node('go_forward_node')
    pub0 = rospy.Publisher('/motor_1/target_duty', Float32, queue_size=1)
    pub1 = rospy.Publisher('/motor_2/target_duty', Float32, queue_size=1)
    time.sleep(1)

    pub0.publish(Float32(0.99))
    pub1.publish(Float32(0.99))
    time.sleep(1.5)
    pub0.publish(Float32(0.0))
    pub1.publish(Float32(0.0))


if __name__ == '__main__':
    main()
