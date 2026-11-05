import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/traj.csv"
f = open(out, "w")
f.write("t,x,y,z\n")


class Rec(Node):
    def __init__(self):
        super().__init__("traj_recorder")
        self.create_subscription(Odometry, "/Odometry", self.cb, 50)

    def cb(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        f.write(f"{t:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f}\n")


rclpy.init()
n = Rec()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
f.close()
