#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

def main():
    rospy.init_node('range_faker', anonymous=True)
    rate = rospy.Rate(1) # 1Hz

    frames_in = rospy.get_param('~frames_in')
    topic_out = rospy.get_param('~topic_out')

    pub = rospy.Publisher(topic_out, Range, queue_size=10)
    
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        
        for frame in frames_in:
            msg = Range()
            
            # defaults
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 1
            msg.min_range = 0.1
            msg.max_range = 0.8
            msg.header.stamp = t
            
            # custom
            msg.header.frame_id = frame
            msg.range = 0.7
        
            pub.publish(msg)
        rate.sleep()

    pub.unregister()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
