import heapq
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray



class AStarNode:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost


def a_star(start, goal, grid): #Path Planning Method
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, (0, start))

    while open_list:
        _, current_node = heapq.heappop(open_list)
        if (current_node.x, current_node.y) == (goal.x, goal.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]

        closed_list.add((current_node.x, current_node.y))
        neighbors = get_neighbors(current_node, grid)

        for neighbor in neighbors:
            if (neighbor.x, neighbor.y) in closed_list:
                continue
            heapq.heappush(open_list, (neighbor.cost, neighbor))

    return []  # Return an empty path if no path is found


def get_neighbors(node, grid):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    neighbors = []

    for dx, dy in directions:
        new_x, new_y = node.x + dx, node.y + dy
        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0:
            cost = node.cost + 1  # Uniform cost for each step
            neighbors.append(AStarNode(new_x, new_y, cost, node))

    return neighbors


class PathPlanner(Node): #Program Start
    def __init__(self):
        super().__init__('path_planner')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) #Publisher for movement
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_markers', 10) #Publisher for points in grid
        
        self.beacon_subscriber = self.create_subscription(Point, '/beacon_data', self.update_beacons, 10) #Subscriber for the beacon in environment
        
        self.robot_position = [0.0, 0.0]
        self.robot_theta = 0.0
        self. pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10) #Publish poses for the Robot and Poses in RVIZ
        self.timer = self.create_timer(0.5, self.publish_path)
        
        #Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9] (Comment out all lines for unused grid map)
        #self.gridSimple = [
        #    [0, 0, 1, 0, 0],
        #    [0, 1, 1, 0, 0],
        #    [0, 0, 0, 0, 1],
        #    [1, 0, 1, 0, 0],
        #    [0, 0, 0, 1, 0],
        #]# 5x5 Simple Grid
        
        self.gridComplex = [
             [0, 0, 0, 0, 1, 1, 0, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 1, 0],
             [1, 0, 1, 1, 1, 0, 0, 0, 1],
             [0, 0, 1, 1, 1, 0, 1, 0, 1],
             [0, 0, 0, 0, 0, 1, 0, 0, 0],
             [0, 1, 0, 1, 0, 0, 1, 1, 0],
             [0, 1, 1, 1, 1, 0, 0, 1, 0],
             [0, 1, 0, 1, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, 1, 1, 0],
             [0, 1, 0, 1, 1, 0, 0, 0, 0],
        ]# 10x9 Complex Grid
        
        self.start = AStarNode(0, 0, 0) #Start Point
        self.goal = AStarNode(9, 8, 0) #Goal Point
        self.beacons = [] #Beacon waypoints
        self.path = [] #Path for robot
        #self.path = a_star(self.start, self.goal, self.grid) #PathPlanning with A*
		
        #if self.path:
        #    self.get_logger().info(f"Path found: {self.path}")
        #else:
        #    self.get_logger().warn("No path found!")

        self.current_index = 0
        
    def update_beacons(self, msg):
    	beacon_x = int(round(msg.x))
    	beacon_y = int(round(msg.y))
    	self.beacons = [(beacon_x, beacon_y)]
    	
    	#Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
    	#if not (0 <= beacon_x < len(self.gridSimple) and 0 <= beacon_y < len(self.gridSimple[0])):
    	if not (0 <= beacon_x < len(self.gridComplex) and 0 <= beacon_y < len(self.gridComplex[0])):
    		self.get_logger().warn(f"Beacon at ({beacon_x}, {beacon_y}) is out of grid bounds")
    		return
    		
    	#Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
    	#if self.gridSimple[beacon_x][beacon_y] == 1:
    	if self.gridComplex[beacon_x][beacon_y] == 1:
    		self.get_logger().warn(f"Beacon at ({beacon_x}, {beacon_y}) is on an obstacle")
    		return
    	else:
    		self.get_logger().info(f"Beacons updated: {self.beacons}")
    		self.beaconPathPlanner()
    
    	
    def beaconPathPlanner(self):
        self.path = []
        current_node = self.start
        for beacon in self.beacons:
            beacon_node = AStarNode(int(beacon[0]), int(beacon[1]), 0)
            
            #Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
            #beacon_path = a_star(current_node, beacon_node, self.gridSimple)
            beacon_path = a_star(current_node, beacon_node, self.gridComplex)
            
            if beacon_path:
               self.path.extend(beacon_path[:-1])
               current_node = beacon_node
            else:
               self.get_logger().warn(f"No path found to the beacon at {beacon}")
        
        #Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
        #beacon_to_goal = a_star(current_node, self.goal, self.gridSimple)
        beacon_to_goal = a_star(current_node, self.goal, self.gridComplex)
        
        if beacon_to_goal:
           self.path.extend(beacon_to_goal)
        self.get_logger().info(f"Beacon Path Planned: {self.path}")
        self.publish_markers()
        self.publish_path()
        
	
    def publish_markers(self):
    	marker_array = MarkerArray()
    	
    	#Plot Grid Points in RVIZ
    	#Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
    	#Uncomment for Simple 5x5 Grid Map
    	#for x in range(len(self.gridSimple)):
    	#   for y in range(len(self.gridSimple[0])):
    	
    	#Uncomment for Complex 10x9 Map
    	for x in range(len(self.gridComplex)):
    	   for y in range(len(self.gridComplex[0])):
    	   
    	   	marker = Marker()
    	   	marker.header.frame_id = "map"
    	   	marker.type = Marker.CUBE
    	   	marker.action = Marker.ADD
    	   	marker.ns = "grid"
    	   	
    	   	#Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
    	   	#marker.id = x * len(self.gridSimple[0]) + y
    	   	marker.id = x * len(self.gridComplex[0]) + y
    	   	
    	   	#Size and Color
    	   	marker.scale.x = 1.0
    	   	marker.scale.y = 1.0
    	   	marker.scale.z = 0.1
    	   	marker.color.r = 0.0
    	   	marker.color.g = 0.0
    	   	
    	   	#Uncomment for Specific map: gridSimple [5x5] | gridComplex [10x9]
    	   	#marker.color.b = 1.0 if self.gridSimple[x][y] == 1 else 0.5
    	   	marker.color.b = 1.0 if self.gridComplex[x][y] == 1 else 0.5
    	   	
    	   	
    	   	marker.color.a = 0.5
    	   	marker.pose.position.x = float(x)
    	   	marker.pose.position.y = float(y)
    	   	marker.pose.position.z = 0.0
    	   	marker_array.markers.append(marker)
    	   	
    	#Plot Start Point [Green]
    	start_marker = Marker()
    	start_marker.header.frame_id = "map"
    	start_marker.type = Marker.SPHERE
    	start_marker.action = Marker.ADD
    	start_marker.ns = "start"
    	start_marker.id = 0
    	start_marker.scale.x = 0.7
    	start_marker.scale.y = 0.7
    	start_marker.scale.z = 0.7
    	start_marker.color.r = 0.0
    	start_marker.color.g = 1.0
    	start_marker.color.b = 0.0 
    	start_marker.color.a = 1.0
    	start_marker.pose.position.x = float(self.start.x)
    	start_marker.pose.position.y = float(self.start.y)
    	start_marker.pose.position.z = 0.0
    	marker_array.markers.append(start_marker)
    	
    	#Plot Goal Point [Red]
    	goal_marker = Marker()
    	goal_marker.header.frame_id = "map"
    	goal_marker.type = Marker.SPHERE
    	goal_marker.action = Marker.ADD
    	goal_marker.ns = "goal"
    	goal_marker.id = 0
    	goal_marker.scale.x = 0.7
    	goal_marker.scale.y = 0.7
    	goal_marker.scale.z = 0.7
    	goal_marker.color.r = 1.0
    	goal_marker.color.g = 0.0
    	goal_marker.color.b = 0.0 
    	goal_marker.color.a = 1.0
    	goal_marker.pose.position.x = float(self.goal.x)
    	goal_marker.pose.position.y = float(self.goal.y)
    	goal_marker.pose.position.z = 0.0
    	marker_array.markers.append(goal_marker)
        
    	#Plot Beacon [Orange/red]
    	for idx, beacon in enumerate(self.beacons):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.ns = "beacons"
            marker.id = idx
            marker.scale.x = 0.7
            marker.scale.y = 0.7
            marker.scale.z = 0.7
            marker.color.r = 3.0
            marker.color.g = 1.0
            marker.color.b = 0.0 
            marker.color.a = 1.0
            marker.pose.position.x = float(beacon[0])
            marker.pose.position.y = float(beacon[1])
            marker.pose.position.z = 0.0
            marker_array.markers.append(marker)
    	    
    	#Plot Path [White]
    	for idx, waypoint in enumerate(self.path):
    	    marker = Marker()
    	    marker.header.frame_id = "map"
    	    marker.type = Marker.SPHERE
    	    marker.action = Marker.ADD
    	    marker.ns = "path"
    	    marker.id = idx
    	    marker.scale.x = 0.3
    	    marker.scale.y = 0.3
    	    marker.scale.z = 0.3
    	    marker.color.r = 1.0
    	    marker.color.g = 1.0
    	    marker.color.b = 1.0 
    	    marker.color.a = 1.0
    	    marker.pose.position.x = float(waypoint[0])
    	    marker.pose.position.y = float(waypoint[1])
    	    marker.pose.position.z = 0.0
    	    marker_array.markers.append(marker)
    	self.marker_publisher.publish(marker_array)
	
    def publish_path(self):
        if self.current_index < len(self.path):
            target_point = self.path[self.current_index]
            target_x, target_y = target_point
            
            #Movement Calulation
            dx = target_x - self.robot_position[0]
            dy = target_y - self.robot_position[1]
            distance = (dx**2 + dy**2)**0.5
            
            if distance < 0.1:
               self.get_logger().info(f"Reached waypoint: {target_point}")
               self.current_index += 1
            else:
               step_size = 0.1 #Movement Speed
               angle_to_target = math.atan2(dy,dx)
               self.robot_position[0] += step_size * math.cos(angle_to_target)
               self.robot_position[1] += step_size * math.sin(angle_to_target)
               self.robot_theta = angle_to_target
               
            #Robot Pose   
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = self.robot_position[0]
            pose_msg.pose.position.y = self.robot_position[1]
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.z = math.sin(self.robot_theta / 2.0)
            pose_msg.pose.orientation.w = math.cos(self.robot_theta / 2.0)
            self.pose_publisher.publish(pose_msg)
                
            #move_cmd = Twist()
            #move_cmd.linear.x = 0.5  # Adjust speed as necessary
            #self.publisher.publish(move_cmd)
            
            #Robot [Yellow]
            robot_marker = Marker()
            robot_marker.header.frame_id = "map"
            robot_marker.type = Marker.SPHERE
            robot_marker.action = Marker.ADD
            robot_marker.ns = "robot"
            robot_marker.id = 999
            robot_marker.scale.x = 0.5
            robot_marker.scale.y = 0.5
            robot_marker.scale.z = 0.5
            robot_marker.color.r = 1.0
            robot_marker.color.g = 1.0
            robot_marker.color.b = 0.0
            robot_marker.color.a = 1.0
            robot_marker.pose.position.x = self.robot_position[0]
            robot_marker.pose.position.y = self.robot_position[1]
            robot_marker.pose.position.z = 0.0
            self.marker_publisher.publish(MarkerArray(markers=[robot_marker]))
            
            self.get_logger().info(f"Moving to {self.path[self.current_index]}")
            #self.current_index += 1
        if self.current_index >= len(self.path):
            move_cmd = Twist()  # Stop the robot
            self.publisher.publish(move_cmd)
            self.get_logger().info("Path complete. Goal Reached!")
            self.timer.cancel()
            return

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
