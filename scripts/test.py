#!/usr/bin/env python3
import rospy




# publisher type trajectory_msgs/MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
# publisher type geometry_msgs/Transform

pub = rospy.Publisher('/red/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=10)


def main():
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.points = [MultiDOFJointTrajectoryPoint(), MultiDOFJointTrajectoryPoint(), MultiDOFJointTrajectoryPoint(), MultiDOFJointTrajectoryPoint(), MultiDOFJointTrajectoryPoint(), MultiDOFJointTrajectoryPoint()]
        msg.points[0].transforms = [Transform()]
        msg.points[1].transforms = [Transform()]
        msg.points[2].transforms = [Transform()]
        msg.points[3].transforms = [Transform()]
        msg.points[4].transforms = [Transform()]
        msg.points[5].transforms = [Transform()]
        msg.points[0].transforms[0].translation.z = 2.10
        msg.points[1].transforms[0].translation.z = 2.20
        msg.points[2].transforms[0].translation.z = 2.30
        msg.points[3].transforms[0].translation.z = 2.40
        msg.points[4].transforms[0].translation.z = 2.50
        msg.points[5].transforms[0].translation.z = 2.60
        pub.publish(msg)
        rate.sleep()
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass