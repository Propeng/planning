import rospy
from autoware_msgs.msg import LaneArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

def get_color(cost):
    if cost >= 0 and cost <= 0.5:
        return ColorRGBA(r=cost*2, g=1, b=0, a=1)
    elif cost > 0.5 and cost <= 1:
        return ColorRGBA(r=1, g=1-cost*2, b=0, a=1)
    else:
        return ColorRGBA(r=1, g=1, b=1, a=1)

def get_waypoints(lane_waypoints):
    markers_array = MarkerArray()
    marker = Marker()
    markers_array.markers.append(marker)
    marker.header.frame_id = '/map'
    marker.ns = 'waypoint_cost'
    marker.id = 0
    marker.action = Marker.MODIFY
    marker.type = Marker.SPHERE_LIST
    marker.scale = Vector3(x=1, y=1, z=1)

    for lane in lane_waypoints.lanes:
        for waypoint in lane.waypoints:
            marker.points.append(waypoint.pose.pose.position)
            marker.colors.append(get_color(waypoint.cost))
    
    pub.publish(markers_array)

rospy.init_node('waypoint_cost_marker')
rospy.Subscriber('/lane_waypoints_array', LaneArray, get_waypoints)
pub = rospy.Publisher('/global_waypoints_mark', MarkerArray, queue_size=10)
rospy.spin()