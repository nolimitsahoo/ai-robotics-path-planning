import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry



class AStarNode:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost


def a_star(start, goal, grid):
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


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) #Publisher for movement
        self.beacon_subscriber = self.create_subscription(Point, '/beacon_data', self.update_beacons, 10) #Subscriber for the beacons in environment
        self.timer = self.create_timer(1.0, self.publish_path)
        self.grid = [
            [0, 0, 1, 0, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 1],
            [1, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
        ]
        self.start = AStarNode(0, 0, 0) #Start 
        self.goal = AStarNode(4, 4, 0) #Goal
        self.beacons = [] #Beacon waypoints
        self.path = [] #path for robot
        #self.path = a_star(self.start, self.goal, self.grid) #PathPlanning with A*
		
        if self.path:
            self.get_logger().info(f"Path found: {self.path}")
        else:
            self.get_logger().warn("No path found!")

        self.current_index = 0
        
    def update_beacons(self, msg):
	beacon_x = int(round(msg.x))
    	beacon_y = int(round(msg.y))
    	self.beacons = [(beacon_x, beacon_y)]
    	self.get_logger().info(f"Beacons updated: {self.beacons}")
    	self.beaconPathPlanner()
    	
    def beaconPathPlanner(self):
    	self.path = []
    	current_node = self.start
    	for beacon in self.beacons:
    		beacon_node = AStarNode(int(beacon[0]), int(beacon[1]), 0)
    		self.path.extend(a_star(current_node, beacon_node, self.grid)[:-1])
    		current_node = beacon_node
    	self.path.extend(a_star(current_node, self.goal, self.grid))
    	self.get_logger().info(f"Beacon Path Planned: {self.path}")

    def publish_path(self):
        if self.current_index < len(self.path):
            move_cmd = Twist()
            move_cmd.linear.x = 0.5  # Adjust speed as necessary
            self.publisher.publish(move_cmd)
            self.get_logger().info(f"Moving to {self.path[self.current_index]}")
            self.current_index += 1
        else:
            move_cmd = Twist()  # Stop the robot
            self.publisher.publish(move_cmd)
            self.get_logger().info("Path complete")
            self.timer.cancel()


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
