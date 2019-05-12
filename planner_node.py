import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from planning_functions import a_star

# node configuration
NODE_NAME = "planner"
ODOM_TOPIC = "/ekf/Odometry" # nav_msgs/Odommetry
MAP_TOPIC = "/rtabmap/grid_map" # nav_msgs/OccupancyGrid
OBJECTIVE_TOPIC = "/objective" # nav_msgs/NavSatFix
WAYPOINT_TOPIC = "/planner/waypoints" # probably nav_msgs/NavSatFix[]

QUEUE_SIZE = 10
PUBLISH_FREQUENCY = 10
LOG_LEVEL = rospy.DEBUG

# node's internal state, all data required for planning
state_current_pose = None
state_current_objective = None
state_grid_origin_pose = None
state_occupancy_grid = None

def is_ready():
    """
    Checks that the node has received sufficient data to begin planning -
    namely, the rover's current pose, and the occupancy grid.
    """
    return state_current_pose is not None and \
           state_current_objective is not None and \
           state_grid_origin_pose is not None and \
           state_occupancy_grid is not None

def pose_to_gridcell(pose):
    """
    Converts a pose (x,y) tuple, assumed to be in the "map" frame to a
    cell coordinate (x,y) in the occupancy grid.
    """
    return (0, 0)

def latlon_to_gridcell(latlon):
    """
    Converts a latlon (lat, lon) tuple to a cell coordinate (x,y) in the
    occupancy grid.
    """
    return (0, 0)

def preprocess_occupancy_grid(raw_grid):
    """
    Preprocesses the raw occupancy grid:
        (1) Reshapes the 1D raw array into a 2D matrix;
        (2) Switches any cell with an obstacle probability above some
            threshold to cost of float("inf");
        (3) TODO: k x k 2D max convolution with "same" padding (this is
            so that the rover doesn't try to pass through cells, with a
            smaller area than the rover itself, which might be adjacent to
            obstacles).
    """
    return raw_grid

def gridcell_to_pose(cell):
    """
    Converts an (x,y) cell coordinate into a pose in the "map" frame.
    """
    return (0, 0)

def odom_callback(msg):
    """
    Extracts the current pose of the rover, ideally in the "map" frame (but
    need to check if this is possible). Used to locate the rover's location
    in the occupancy grid.
    """
    state_current_pose = msg.pose
    state_current_twist = msg.twist

def map_callback(msg):
    """
    Extracts and preprocesses the most recent version of the occupancy
    grid.
    """
    state_grid_origin_pose = msg.info.origin
    state_occupancy_grid = msg.data

def objective_callback(msg):
    """
    Saves the current objective to the node's internal state.
    """
    state_objective_pose = gps_to_pose(msg.latitude, msg.longitude)

def planner():
    """
    The "main" function for the rosnode. Initializes the node and runs the
    main control loop.
    """
    # node initalization
    rospy.init_node(NODE_NAME, log_level=LOG_LEVEL)

    # subscribe to odom and grid_map
    odom_subscriber = rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
    grid_map_subscriber = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)
    objective_subscriber = rospy.Subscriber(OBJECTIVE_TOPIC, NavSatFix, objective_callback)

    # main control loop - generate waypoints over occupancy grid and publish
    waypoint_publisher = rospy.Publisher(WAYPOINT_TOPIC, String, queue_size=QUEUE_SIZE)
    publish_rate = rospy.Rate(PUBLISH_FREQUENCY)
    while not rospy.is_shutdown():
        if is_ready() or True:
            rospy.logdebug("Planner ready to publish.")
            start = pose_to_gridcell(state_current_pose)
            end = latlon_to_gridcell(state_current_objective)
            grid = preprocess_occupancy_grid(state_occupancy_grid)
            waypoints = [gridcell_to_pose(wp) for wp in a_star(grid, start, end]
            waypoint_publisher.publish(str(waypoints))
        else:
            rospy.logdebug("Planner not ready to publish.")
        publish_rate.sleep()

if __name__ == "__main__":
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
