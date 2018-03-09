import rospy
from std_msgs.msg import String

rospy.init_node('press')
while rospy.Time().now().to_sec() == 0:
    pass

pub = rospy.Publisher('pressure', String, queue_size=10)
while True:
    st = String()
    st.data = 'start'
    pub.publish(st)
    rospy.sleep(1)
