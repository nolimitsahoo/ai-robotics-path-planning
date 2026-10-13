
import heapq
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Node:
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
            neighbors.append(Node(new_x, new_y, cost, node))

    return neighbors

def main():
    rospy.init_node('path_planner')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Example grid (0 = free, 1 = obstacle)
    grid = [
        [0, 0, 1, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 1],
        [1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
    ]

    start = Node(0, 0, 0)
    goal = Node(4, 4, 0)

    path = a_star(start, goal, grid)

    if path:
        rospy.loginfo("Path found: %s", path)
        move_robot(path, pub)
    else:
        rospy.logwarn("No path found!")

def move_robot(path, pub):
    rate = rospy.Rate(1)
    for x, y in path:
        move_cmd = Twist()
        move_cmd.linear.x = 0.5  # Adjust speed as necessary
        pub.publish(move_cmd)
        rospy.sleep(1)  # Simulate moving to the next point
    
    # Stop the robot
    move_cmd = Twist()
    pub.publish(move_cmd)

def odometry_callback(msg):
    pass  # Implement as needed to track the robot's position

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
