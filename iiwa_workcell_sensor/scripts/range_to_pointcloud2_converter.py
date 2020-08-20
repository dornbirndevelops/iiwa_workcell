#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Range, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf import TransformListener, LookupException

class RangeToPointCloud2Converter(object):
    def __init__(self, frame_out):
        self.tl = TransformListener()
        self.points = []
        self.frame_out = frame_out

    def receiveRange(self, msg):
        ps = PointStamped()
        ps.header.frame_id = msg.header.frame_id
        ps.header.stamp = rospy.Time(0)
        ps.point.x = min(max(msg.range, msg.min_range), msg.max_range)
        
        try:
            p = self.tl.transformPoint(self.frame_out, ps).point
            xyz = [p.x, p.y, p.z]
            self.points.append(xyz)
        except LookupException as e:
            rospy.logerr('could not find transform from {} to {}'.format(ps.header.frame_id, self.frame_out))

    def flushPointCloud(self):
        header = Header()
        header.frame_id = self.frame_out
        header.stamp = rospy.Time.now()

        pc = create_cloud_xyz32(header, self.points) if len(self.points) else None
        self.points = []
        return pc

def main():
    rospy.init_node('range_to_pointcloud2', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    
    topics_in = rospy.get_param('~topics_in')
    topic_out = rospy.get_param('~topic_out')
    frame_out = rospy.get_param('~frame_out')

    converter = RangeToPointCloud2Converter(frame_out)

    subs = [rospy.Subscriber(topic_in, Range, converter.receiveRange) for topic_in in topics_in]
    pub = rospy.Publisher(topic_out, PointCloud2, queue_size=10)

    while not rospy.is_shutdown():
        pc = converter.flushPointCloud()
        if pc:
            pub.publish(pc)
        else:
            rospy.loginfo('got no pointcloud data, skipping publishment')
        rate.sleep()
    
    for sub in subs:
        sub.unregister()
    pub.unregister()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
