+from threading import Lock
import math

import rospy
from autoware_msgs.msg import LaneArray, Lane
from geometry_msgs.msg import Pose, PoseStamped

from dstarlite import DStarLite

lane_array = LaneArray()
current_pose = Pose()
curr_lane_i = curr_waypoint_i = 0

waypoints = []
path = []
pf = None

lock = Lock()

def update_pf():
    global pf, curr_lane_i, curr_waypoint_i, path

    if waypoints and current_pose:
        if not pf:
            pf = DStarLite(len(waypoints[0]), len(waypoints)-1)
            pf.compute_shortest_path()
    
        #Get waypoint we are closest to
        curr_lane_i = 0
        curr_waypoint_i = 0
        min_dist = None
        for lane_i, lane in enumerate(lane_array.lanes):
            for waypoint_i, waypoint in enumerate(lane.waypoints):
                waypoint_pos = waypoint.pose.pose.position
                our_pos = current_pose.position
                dx = our_pos.x - waypoint_pos.x
                dy = our_pos.y - waypoint_pos.y
                dist = math.sqrt(dx**2 + dy**2)
                if not min_dist or dist <= min_dist:
                    min_dist = dist
                    curr_lane_i = lane_i
                    curr_waypoint_i = waypoint_i
        
        pf.s_start = (curr_waypoint_i, curr_lane_i)
        pf.compute_shortest_path()

        path = pf.step(look_ahead=20)

def update_lane_waypoints(msg_lane_array):
    global lane_array
    global waypoints
    lane_array = msg_lane_array

    lock.acquire()
    try:
        new_waypoints = []
        waypoint_i = 0#curr_waypoint_i
        while True:
            alternatives = []
            has_waypoints = False
            for lane in lane_array.lanes:
                if waypoint_i < len(lane.waypoints):
                    alternatives.append(lane.waypoints[waypoint_i])
                    has_waypoints = True
                else:
                    alternatives.append(None)
            if not has_waypoints:
                break
            waypoint_i += 1
            new_waypoints.append(alternatives)

        old_waypoints = waypoints
        waypoints = new_waypoints
        update_pf()
        
        for wp_i, wp in enumerate(new_waypoints):
            for l_i, l in enumerate(wp):
                old_cost = None
                if wp_i < len(old_waypoints) and l_i < len(old_waypoints[wp_i]):
                    old_cost = old_waypoints[wp_i][l_i].cost
                new_cost = l.cost
                if old_cost != new_cost:
                    pf.set_cost((wp_i, l_i), new_cost)
    finally:
        lock.release()

def update_pose(msg_pose):
    global current_pose
    current_pose = msg_pose.pose
    
    lock.acquire()
    try:
        update_pf()
    finally:
        lock.release()

def create_msg():
    global waypoints

    if not pf or not waypoints:
        return None
    
    lock.acquire()
    try:
        # look_ahead = curr_waypoint_i + 20
        # if look_ahead > len(waypoints):
        #     look_ahead = len(waypoints)

        # final_waypoints = []
        # final_waypoints.append(waypoints[curr_waypoint_i][curr_lane_i])
        # for i in range(curr_waypoint_i+1, look_ahead):
        #     alternatives = waypoints[i]
        #     min_cost = min(alternatives, key=lambda p: p.cost)
        #     final_waypoints.append(min_cost)

        #print(path)

        final_waypoints = []
        for node in path:
            try:
                final_waypoints.append(waypoints[node[0]][node[1]])
            except IndexError:
                #path outside waypoints
                break

        lane = Lane()
        lane.waypoints = final_waypoints

        return lane
    finally:
        lock.release()

rospy.init_node('waypoint_astar')
rospy.Subscriber('/lane_waypoints_array', LaneArray, update_lane_waypoints)
rospy.Subscriber('/current_pose', PoseStamped, update_pose)
pub = rospy.Publisher('/final_waypoints', Lane, queue_size=10)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = create_msg()
    if msg:
        pub.publish(msg)
    try:
        r.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        pass
