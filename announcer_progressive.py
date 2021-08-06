import math
import time
from collections import defaultdict
from threading import Thread

import rospy
import tf
from autoware_msgs.msg import LaneArray, Lane, Waypoint
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Vector3, Point, TwistStamped
from std_msgs.msg import ColorRGBA, Float32

from conv import apply_conv

offline_map = []
current_waypoint = None
current_lane_waypoint = None
current_pose = None
current_velocity = 0

lane_width = 2.5
#road_width_measurements = defaultdict(lambda: (0, 10))
road_width_measurements = defaultdict(lambda: (1.3, 1.3))

obstacles = []
moving_obstacles = []

class AddObstacleThread(Thread):
    def run(self):
        global moving_obstacles

        while True:
            dist = input('Obstacle distance from current waypoint: ')
            new_waypoint = slide_waypoint(current_waypoint, dist, along_normal=False)
            position = new_waypoint.pose.pose.position
            obstacles.append((position.x, position.y, 0.5))
            print("Added obstacle at (%f, %f)" % (position.x, position.y))
            continue

            velocity = 2
            sim_duration = 10
            time_step = 0.1

            ori = current_waypoint.pose.pose.orientation
            (_, _, yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
            yaw -= math.pi / 2

            new_waypoint = slide_waypoint(current_waypoint, dist, along_normal=False)
            new_waypoint = slide_waypoint(new_waypoint, -velocity*3, along_normal=True)

            for _ in range(int(sim_duration/time_step)):
                new_waypoint = slide_waypoint(new_waypoint, velocity*time_step, along_normal=True)
                position = new_waypoint.pose.pose.position
                moving_obstacles = [(position.x, position.y, yaw, velocity)]
                time.sleep(time_step)
            moving_obstacles = []

AddObstacleThread().start()

def set_yaw(waypoints):
    yaw = 0
    for i in range(len(waypoints)):
        point = waypoints[i]
        if i < len(waypoints)-1:
            next_point = waypoints[i+1]
            #Get yaw
            dy = next_point.pose.pose.position.y - point.pose.pose.position.y
            dx = next_point.pose.pose.position.x - point.pose.pose.position.x
            yaw = math.atan2(dy, dx)

        ori = waypoints[i].pose.pose.orientation
        (ori.x, ori.y, ori.z, ori.w) = tf.transformations.quaternion_from_euler(0, 0, yaw)

def slide_waypoint(waypoint, dist, along_normal=True):
    pos = waypoint.pose.pose.position
    (x, y, z) = (pos.x, pos.y, pos.z)
    ori = waypoint.pose.pose.orientation
    (_, _, yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))

    #Move the point dist units along the normal
    if along_normal:
        yaw_normal = yaw - math.pi/2
        x += dist * math.cos(yaw_normal)
        y += dist * math.sin(yaw_normal)
    else:
        x += dist * math.cos(yaw)
        y += dist * math.sin(yaw)

    new_waypoint = Waypoint()
    new_waypoint.time_cost = waypoint.time_cost
    new_waypoint.pose.pose.orientation = waypoint.pose.pose.orientation #new waypoint has the same yaw
    new_pos = new_waypoint.pose.pose.position
    (new_pos.x, new_pos.y, new_pos.z) = (x, y, z)

    return new_waypoint

def find_intersection(x0_1, y0_1, yaw1, vel1, x0_2, y0_2, yaw2, vel2):
    xt1 = vel1 * math.cos(yaw1)
    yt1 = vel1 * math.sin(yaw1)
    xt2 = vel2 * math.cos(yaw2)
    yt2 = vel2 * math.sin(yaw2)

    #x1 = x2  -->  x0_1 + t1*xt_1 = x0_2 + t2*xt_2
    #y1 = y2  -->  y0_1 + t1*yt_1 = y0_2 + t2*yt_2

    #We solve the following system using cramer's method
    #t1*xt1 - t2*xt2 = x0_2 - x0_1
    #t1*yt1 - t2*yt2 = y0_2 - y0_1
    
    d = -yt2 * xt1 + yt1 * xt2
    dt1 = -yt2 * (x0_2 - x0_1) + xt2 * (y0_2 - y0_1)
    dt2 = xt1 * (y0_2 - y0_1) - yt1 * (x0_2 - x0_1)

    t1 = dt1 / d
    t2 = dt2 / d
    return (t1, t2)

def load_points_src(waypoints_src_file):
    global offline_map
    with open(waypoints_src_file, 'r') as f:
        lines = [line for line in f.read().split('\n') if line != '']
    points_s = [line.split(',') for line in lines]
    points = [[float(v) for v in p] for p in points_s]
    #load one in every n_skip waypoints to speed up rviz
    n_skip = 2
    waypoints_src = []
    for i, p in enumerate(points):
        if i % n_skip == 0:
            waypoints_src.append(tuple(p[0:3]))

    waypoints = []
    for waypoint_tuple in waypoints_src:
        waypoint = Waypoint()
        pos = waypoint.pose.pose.position
        (pos.x, pos.y, pos.z) = waypoint_tuple
        waypoint.time_cost = 2.0
        waypoints.append(waypoint)
    
    for i in range(1, 4):
        waypoints[-i].time_cost = 0
    
    return waypoints

offline_map = load_points_src('/home/pc2/Desktop/giu_waypoints.csv')
set_yaw(offline_map)
#offline_map = []

def get_offline_map(lane):
    offline_map = [waypoint for waypoint in lane.waypoints]
    set_yaw(offline_map)

def get_closest_waypoint(pose, waypoints):
    pos = pose.pose.position
    min_dist = None
    min_dist_waypoint = None
    ori = pose.pose.orientation
    (_, _, vehicle_yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
    for waypoint in waypoints:
        dx = waypoint.pose.pose.position.x - pos.x
        dy = waypoint.pose.pose.position.y - pos.y
        dist = math.sqrt(dx**2 + dy**2)
        #if min_dist and dist > min_dist:
        #    return min_dist_waypoint
        #else:
        #    min_dist = dist
        #    min_dist_waypoint = waypoint
        ori = waypoint.pose.pose.orientation
        (_, _, waypoint_yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
        yaw_diff = waypoint_yaw - vehicle_yaw
        while yaw_diff >= math.pi * 2.0:
            yaw_diff -= math.pi * 2.0
        while yaw_diff <= -math.pi * 2.0:
            yaw_diff += math.pi * 2.0
        if (abs(yaw_diff) < math.pi/2.0 or abs(yaw_diff) > 3*math.pi/2.0) and (not min_dist or dist < min_dist):
            min_dist = dist
            min_dist_waypoint = waypoint
    return min_dist_waypoint

def get_current_waypoint(pose):
    global current_waypoint
    global current_pose
    current_pose = pose
    current_waypoint = get_closest_waypoint(pose, offline_map)

def get_current_velocity(msg):
    global current_velocity
    #current_velocity = msg.twist.linear.x
    current_velocity = msg.data

def create_msg():
    global current_lane_waypoint

    if not offline_map or not current_waypoint:
        return LaneArray()
    
    start_i = offline_map.index(current_waypoint)
    end_i = start_i + 20
    look_ahead_waypoints = offline_map[start_i:end_i]

    lane_arr = LaneArray()

    #n_lanes = 3
    #lane_arr.lanes = [Lane() for i in range(n_lanes)]
    #lane_arr.lanes = []

    alternatives_list = []

    for i, waypoint in enumerate(look_ahead_waypoints):
        alternatives = []
        alternatives_list.append(alternatives)

        #open_lane_i = (i+start_i) % (n_lanes*2)
        #if open_lane_i >= n_lanes:
        #    open_lane_i = n_lanes*2 - open_lane_i - 1
        
        #get driveable area to the left and right of waypoint
        left_dist, right_dist = road_width_measurements[waypoint]
        #maximum number of possible waypoints
        waypoint_n_lanes = int((left_dist + right_dist) // lane_width)

        center_offset = (-left_dist + right_dist) / 2.0
        leftmost_lane_start = center_offset - waypoint_n_lanes*lane_width/2

        for i in range(waypoint_n_lanes):
            lane_waypoint = slide_waypoint(waypoint, leftmost_lane_start + lane_width*(i + 0.5))
            #lane_waypoint.cost = 0 if i == open_lane_i else 1

            waypoint_x = lane_waypoint.pose.pose.position.x
            waypoint_y = lane_waypoint.pose.pose.position.y
            ori = lane_waypoint.pose.pose.orientation
            (_, _, waypoint_yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
            
            #Check for static obstacles in waypoint range (lane_width radius)
            static_cost = 0
            for (obstacle_x, obstacle_y, obstacle_radius) in obstacles:
                dx = obstacle_x - waypoint_x
                dy = obstacle_y - waypoint_y
                dist = math.sqrt(dx**2 + dy**2)
                if dist <= lane_width/2 + obstacle_radius:
                    static_cost = 1
                    break
            
            #Check for intersection with dynamic obstacles
            dynamic_cost = 0
            for (obstacle_x, obstacle_y, obstacle_yaw, obstacle_vel) in moving_obstacles:
                (t_vehicle, t_obstacle) = find_intersection(
                    waypoint_x, waypoint_y, waypoint_yaw, current_velocity,
                    obstacle_x, obstacle_y, obstacle_yaw, obstacle_vel)
                if t_vehicle >= 0 and t_vehicle <= 1 and t_obstacle >= 0 and t_obstacle <= 10:
                    dynamic_cost = 1

            lane_waypoint.cost = max(static_cost, dynamic_cost)

            #if i >= len(lane_arr.lanes):
            #    lane_arr.lanes.append(Lane())
            #lane_arr.lanes[i].waypoints.append(lane_waypoint)
            alternatives.append(lane_waypoint)
    
    apply_conv(alternatives_list)

    #Convert alternatives list to lane array
    max_lane_count = max(len(alt) for alt in alternatives_list)
    lane_arr.lanes = [Lane() for i in range(max_lane_count)]
    for alternatives in alternatives_list:
        for i, waypoint in enumerate(alternatives):
            lane_arr.lanes[i].waypoints.append(waypoint)

    #Get current lane waypoint
    if current_pose:
        all_lane_waypoints = []
        for alternatives in alternatives_list:
            all_lane_waypoints += alternatives
        current_lane_waypoint = get_closest_waypoint(current_pose, all_lane_waypoints)

    return lane_arr

def create_obstacles_msg():
    markers_array = MarkerArray()

    all_obstacles = []
    all_obstacles += obstacles
    all_obstacles += [(x, y, 1) for (x, y, _, _) in moving_obstacles]

    for i, (x, y, radius) in enumerate(all_obstacles):
        marker = Marker()
        markers_array.markers.append(marker)
        marker.header.frame_id = '/map'
        marker.ns = 'static_obstacle'
        marker.id = i
        marker.action = Marker.MODIFY
        marker.type = Marker.SPHERE_LIST
        marker.scale = Vector3(x=radius*2.0, y=radius*2.0, z=radius*2.0)
        marker.points.append(Point(x=x, y=y, z=current_waypoint.pose.pose.position.z))
        marker.colors.append(ColorRGBA(r=1, g=0, b=0, a=0.25))
    
    return markers_array

def create_velocity_msg():
    if current_lane_waypoint:
        velocity = current_lane_waypoint.time_cost * (1.0 - current_lane_waypoint.cost)
        return Float32(data=velocity)

def create_current_waypoint_msg():
    if current_waypoint:
        return current_waypoint
    elif offline_map and len(offline_map) > 0:
        return offline_map[0]
    else:
        return None

rospy.init_node('announcer_progressive')
rospy.Subscriber('current_pose', PoseStamped, get_current_waypoint)
#rospy.Subscriber('current_velocity', TwistStamped, get_current_velocity)
rospy.Subscriber('estimated_vel_kmph', Float32, get_current_velocity)
pub = rospy.Publisher('lane_waypoints_array', LaneArray, queue_size=10)
velocity_pub = rospy.Publisher('target_velocity', Float32, queue_size=10)
obstacles_pub = rospy.Publisher('obstacles_mark', MarkerArray, queue_size=10)
current_waypoint_pub = rospy.Publisher('current_waypoint', Waypoint, queue_size=10)

freq = 10
r = rospy.Rate(freq)

while not rospy.is_shutdown():
    msg = create_msg()
    pub.publish(msg)
    obstacles_pub.publish(create_obstacles_msg())
    velocity_pub.publish(create_velocity_msg())
    current_waypoint_msg = create_current_waypoint_msg()
    if current_waypoint_msg:
        current_waypoint_pub.publish(current_waypoint_msg)
    try:
        r.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        pass
