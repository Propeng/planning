import rospy
from autoware_msgs.msg import Lane
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

def got_waypoints(lane):
    markers_array = MarkerArray()
    marker = Marker()
    markers_array.markers.append(marker)
    marker.header.frame_id = '/map'
    marker.ns = 'waypoint_path'
    marker.id = 0
    marker.action = Marker.MODIFY
    marker.type = Marker.LINE_STRIP
    marker.scale = Vector3(x=0.2, y=0.2, z=0.2)

    for waypoint in lane.waypoints:
        marker.points.append(waypoint.pose.pose.position)
        #marker.colors.append(ColorRGBA(r=waypoint.cost, g=1-waypoint.cost, b=0, a=1))
        marker.colors.append(ColorRGBA(r=0.2, g=0.2, b=1, a=1))
    
    pub.publish(markers_array)

rospy.init_node('waypoint_path_marker')
rospy.Subscriber('/final_waypoints', Lane, got_waypoints)
pub = rospy.Publisher('/local_waypoints_mark', MarkerArray, queue_size=10)
rospy.spin()