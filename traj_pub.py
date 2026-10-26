#!/usr/bin/env python3
import sys
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node("traj_pub")

fn  = sys.argv[1] if len(sys.argv) > 1 else "CameraTrajectory.txt"
pub = rospy.Publisher("/slam/trajectory", Path, queue_size=1, latch=True)

path = Path()
path.header.frame_id = "map"

try:
    fh = open(fn)
except IOError as e:
    rospy.logfatal("cannot open %s: %s", fn, e)
    sys.exit(1)

n = 0
with fh:
    for line in fh:
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        v = s.split()
        if len(v) < 8:
            continue
        try:
            t = float(v[0])
            x, y, z, qx, qy, qz, qw = map(float, v[1:8])   # TUM/ORB-SLAM order
        except ValueError:
            rospy.logwarn("skipping bad line: %s", s)
            continue

        # normalize quaternion (protects against "twisted" rviz display)
        n2 = qx*qx + qy*qy + qz*qz + qw*qw
        if n2 < 1e-12:
            rospy.logwarn("zero quaternion at t=%.3f, using identity", t)
            qx = qy = qz = 0.0; qw = 1.0
        else:
            inv = 1.0 / math.sqrt(n2)
            qx, qy, qz, qw = qx*inv, qy*inv, qz*inv, qw*inv

        p = PoseStamped()
        p.header.frame_id = path.header.frame_id
        p.header.stamp    = rospy.Time.from_sec(t) if t > 1e6 else rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        path.poses.append(p)
        n += 1

if not path.poses:
    rospy.logerr("no valid poses parsed from %s", fn)
    sys.exit(1)

path.header.stamp = rospy.Time.now()
rospy.loginfo("loaded %d poses from %s -> /slam/trajectory", n, fn)

# Latched topic: publish a few times so late subscribers definitely get it,
# then keep spinning so the node stays alive.
rate = rospy.Rate(1)
for _ in range(3):
    pub.publish(path)
    rate.sleep()
rospy.spin()

