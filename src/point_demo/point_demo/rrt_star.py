#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
import math

# Gazebo State Libraries
from gazebo_msgs.msg import ModelState


from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Current EE start position
# x: 0.6764215674966578
# y: 1.4317488629040372e-06
# z: -0.6141726474246604

# ------------ Replacement for controllers ------------#

# super().__init__('move_sphere')
# self.publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
# self.timer = self.create_timer(0.1, self.move_sphere)

# -----------------------------------------------------#

# Take in a goal end effector position
class positionSubscriber(Node):
    def __init__(self):
        super().__init__("position_subscriber")
        self.subscription = self.create_subscription(
            Point,
            '/current_position',
            self.position_callback,
            10
        )
        self.current_position = None

    def position_callback(self, msg):
        # Update current position with latest EE position message
        self.current_position = msg.x, msg.y, msg.z
        self.get_logger().info(f"Position: x={msg.x}, y={msg.y}, z={msg.z}")
    
    # Make current position accessible to other methods
    def get_current_position(self):
        return self.current_position

# ------------------ RRT* ------------------ #
class RRTStar(Node):
    def __init__(self, start, goal, max_iterations = 50, step_size = 0.01, tol = 0.01):
        # Initialize RRT* Node
        super().__init__("rrt_star")
        self.start = start
        self.goal = goal
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.tree = {tuple(start):None}
        self.tol = tol
        self.radius = 0.5

    # Find random point in the range of start position +/- 0.5 that moves EE closer to the goal
    # def sample_random_point(self):
    #     while True:
    #         random_offset = np.random.uniform(-0.5, 0.5, len(self.start))
    #         random_point = np.array(self.start) + random_offset

    #         # Calculate distances
    #         current_to_goal = np.linalg.norm(np.array(self.start)-np.array(self.goal))
    #         random_to_goal = np.linalg.norm(random_point - np.array(self.goal))

    #         # Check if the random point is closer to the goal than the current position and not in collision
    #         if random_to_goal < current_to_goal and np.sqrt((random_point[0]-3)**2 + (random_point[1])**2 + (random_point[2])**2) >1:
    #             return random_point
    
    # def is_goal_reached(self):
    #     distance_to_goal = np.linalg.norm(np.array(self.start) - np.array(self.goal))
    #     return distance_to_goal<self.step_size
    def tree_builder(self):
        cost = 3*math.dist(self.start, self.goal)
        newPoint = self.start
        iterations =self.max_iterations
        # Iterate until goal is reached and cost is less than twice the distance between start and goal
        # while (np.linalg.norm(np.array(newPoint) - np.array(self.goal))>self.tol) and self.max_iterations!=0:
        while iterations>0:
            self.get_logger().info(f"Iteration: {iterations}")
            iterations -= 1
            # Sample a random point with goal biasing
            # if np.random.uniform(0, 1) < 0.01:
            #     newPoint = self.goal
            #     nearest_point = min(self.tree.keys(), key=lambda p: np.linalg.norm(np.array(p) - np.array(newPoint)))
            #     self.tree[tuple(newPoint)] = nearest_point
            #     self.get_logger().info('Skipped to goal')
            #     # continue
            # else:
            if iterations >= self.max_iterations*2/3:
                newPoint = tuple([np.round(np.random.uniform(0, math.dist(self.start,self.goal)/3),2),
                            np.round(np.random.uniform(-math.dist(self.start,self.goal)/3, math.dist(self.start,self.goal)/3),2),
                            np.round(np.random.uniform(0, math.dist(self.start,self.goal))/2,2)])
            elif iterations >= self.max_iterations/3:
                newPoint = tuple([np.round(np.random.uniform(math.dist(self.start,self.goal)/3,math.dist(self.start,self.goal)*2/3),2),
                            np.round(np.random.uniform(-math.dist(self.start,self.goal)/3, math.dist(self.start,self.goal)/3),2),
                            np.round(np.random.uniform(0, math.dist(self.start,self.goal))/2,2)])
            elif iterations >= 3:
                newPoint = tuple([np.round(np.random.uniform(math.dist(self.start,self.goal)*2/3, math.dist(self.start,self.goal)),2),
                            np.round(np.random.uniform(-math.dist(self.start,self.goal)/3, math.dist(self.start,self.goal)/3),2),
                            np.round(np.random.uniform(0, math.dist(self.start,self.goal))/3,2)])
            else:
                newPoint = tuple([np.round(np.random.uniform(self.goal[0]-self.tol, self.goal[0]+self.tol),2),
                            np.round(np.random.uniform(self.goal[1]-self.tol, self.goal[1]+self.tol),2),
                            np.round(np.random.uniform(self.goal[2]-self.tol, self.goal[2]+self.tol),2)])

            if iterations%100 == 0:
                self.get_logger().info(f'Generated random point:{newPoint}')
            # Find the nearest point in the tree
            nearest_point = min(self.tree.keys(), key=lambda p: np.linalg.norm(np.array(p) - np.array(newPoint)))
            # Check if the new point is in collision.  REPLACE HARD CODED VALUES WITH OBSTACLE POSITION
            # Avoid singularity by setting retry count
            retry_count = 0
            while math.dist([3,0,0], newPoint) <= .15:
                retry_count += 1
                if retry_count > 10:
                    self.get_logger().info('Retrying to find a new point')
                    break
                newPoint = tuple([np.random.uniform(-math.dist(self.start,self.goal), math.dist(self.start,self.goal)),
                        np.random.uniform(-math.dist(self.start,self.goal), math.dist(self.start,self.goal)),
                        np.random.uniform(0, math.dist(self.start,self.goal))])
                # Find the nearest point in the tree
                nearest_point = min(self.tree.keys(), key=lambda p: np.linalg.norm(np.array(p) - np.array(newPoint))+np.linalg.norm(np.array(p) - np.array(self.goal)))
            
            # find neighbors in the radius around the new point
            neighbors = []
            for point in self.tree.keys():
                if np.linalg.norm(np.array(point) - np.array(newPoint)) < self.radius:
                    neighbors.append(point)
            # Find the best parent
            # best_parent = nearest_point
            # min_cost = self.get_cost(nearest_point) + np.linalg.norm(np.array(nearest_point) - np.array(newPoint)) + np.linalg.norm(np.array(self.goal) - np.array(newPoint))
            # for neighbor in neighbors:
            #     current_cost = self.get_cost(neighbor) + np.linalg.norm(np.array(neighbor) - np.array(newPoint))
            #     parent_of_neighbor = self.tree[tuple(neighbor)]
            #     directCost = self.get_cost(parent_of_neighbor) + np.linalg.norm(np.array(parent_of_neighbor) - np.array(newPoint))
            #     if directCost < current_cost:
            #         best_parent = neighbor
            #         min_cost = cost
            # Add the new point to the tree with best neighbor as parent
            if not self.is_descendant(tuple(newPoint), nearest_point):
                self.tree[tuple(newPoint)] = nearest_point
            else:
                self.get_logger().error(f"Cycle detected in the tree. Can't assign {nearest_point} as parent of {newPoint}.")

            # For all neighbors in the radius, check if the new point is a better parent
            for neighbor in neighbors:
                if not self.is_descendant(tuple(neighbor), tuple(newPoint)):
                    current_cost = self.get_cost(neighbor) + np.linalg.norm(np.array(neighbor) - np.array(newPoint))
                    parent_of_neighbor = self.tree[tuple(neighbor)]
                    direct_cost = self.get_cost(parent_of_neighbor) + np.linalg.norm(np.array(parent_of_neighbor) - np.array(newPoint)) 
                    if direct_cost < current_cost:
                        self.tree[tuple(neighbor)] = newPoint
                        self.update_subtree_costs(neighbor)  # Update the costs of all descendants
                else:
                    self.get_logger().error(f"Cycle detected in the tree. Can't assign {newPoint} as parent of {neighbor}.")
            # If new point is within tolerance of the goal, assign the goal as the parent of the new point
            # if np.linalg.norm(np.array(newPoint) - np.array(self.goal)) < self.tol:
            #     self.tree[tuple(self.goal)] = tuple(newPoint)
                # self.get_logger().info(f"Tree: {self.tree}")

        self.get_logger().info("First loop exited")
        # Return the path that satisfies the cost and goal
        # Start from the goal and back track to the start.  Then reverse the path.
        # self.get_logger().info(f'{self.tree.keys()}')
        goalCandidates = [node for node in self.tree.keys() if np.linalg.norm(np.array(node) - np.array(self.goal)) <= self.tol*2]
        for candidate in goalCandidates:
            candidate_cost = self.get_cost(candidate)
            self.get_logger().info(f"Goal candidate: {candidate}, Cost: {candidate_cost}")
        best_goal_node = min(goalCandidates, key=lambda node: self.get_cost(node), default=None)
        self.get_logger().info(f"Best goal node: {best_goal_node}")
        if best_goal_node is None:
            best_goal_node = tuple(self.goal)
            self.tree[tuple(self.goal)] = tuple(best_goal_node)
        path = []
        visited = set()
        current = best_goal_node
        while current is not None:
            if current in visited:
                self.get_logger().error("Cycle detected in the tree. Path construction aborted.")
                break
            visited.add(current)
            path.append(current)
            current = self.tree.get(current)
        path.reverse()
        return path
    
    # def is_goal_reached(self):
    #     distance_to_goal = np.linalg.norm(np.array(self.start) - np.array(self.goal))
    #     return distance_to_goal<self.step_size
    
    # Calculate the cost entire path from the start to the given point
    def get_cost(self, point):
        cost = 0
        current = point
        visited = set()
        while current is not None:
            if current in visited:
                self.get_logger().error("Cycle detected in the tree. Cost calculation aborted.")
                break
            visited.add(current)
            parent = self.tree[tuple(current)]
            if parent is not None:
                cost += np.linalg.norm(np.array(tuple(current)) - np.array(parent))
            current = parent
        return cost
    
    # Update the costs of all descendants of a given point
    def update_subtree_costs(self, point):
        for child, parent in self.tree.items():
            if parent == point:  # If the current node is a child of the given point
                # Update the cost of the child
                self.tree[child] = point
                # Recursively update the costs of the child's descendants
                self.update_subtree_costs(child)
    
    # Prevent circular references in the tree
    def is_descendant(self, node, potential_parent):
        current = potential_parent
        while current is not None:
            if current == node:
                return True
            current = self.tree.get(current)
        return False

def main(args = None):
    rclpy.init(args = args)

    position = positionSubscriber()

    # Wait for positionSubscriber to receive the first message
    while rclpy.ok() and position.get_current_position() is None:
        position.get_logger().info("Waiting for  initial position..")
        rclpy.spin_once(position, timeout_sec = 1.0)

    # get initial EE position
    start_position = position.get_current_position()
    # If no EE position is received, log an error and exit
    if start_position is None:
        position.get_logger().error("Failed to get initial position.")
        return

    # Define goal position
    goal_position = [5,0,0] # Replace with actual goal position

    # Initialize RRT* and plan
    planner = RRTStar(start_position, goal_position)
    
    # Publisher for executing the planned path
    trajectory_publisher = planner.create_publisher(Point, '/next_position',10)
    # next_pos_publisher = planner.create_publisher(ModelState, '/gazebo/set_model_state', 10)

    # Publish points from the RRT* algorithm with a delay
    path = planner.tree_builder()
    planner.get_logger().info(f"Planned path: {path}")
    for point in path:
        point_msg = Point()
        point_msg.x = float(point[0])
        point_msg.y = float(point[1])
        point_msg.z = float(point[2])
        trajectory_publisher.publish(point_msg)
        planner.get_logger().info(f"Published point: {point}")
        time.sleep(.1)
        # Add a delay between publishing points
        rclpy.spin_once(planner, timeout_sec=1.0)
    planner.get_logger().info("Path published successfully!")

    # # Keep sampling random points until the goal is reached
    # while rclpy.ok() and not planner.is_goal_reached():
    #     random_point = planner.sample_random_point()
    #     planner.get_logger().info(f"Sampled random point: {random_point}")
        
    #     #Publish random point
    #     point_msg = Point()
    #     point_msg.x = random_point[0]
    #     point_msg.y = random_point[1]
    #     point_msg.z = random_point[2]
    #     trajectory_publisher.publish(point_msg)
    #     # msg = ModelState()
    #     # msg.model_name = 'your_sphere_model_name'
    #     # msg.pose.position.x = random_point[0]  # Example position
    #     # msg.pose.position.y = random_point[1]
    #     # msg.pose.position.z = random_point[2]
    #     # next_pos_publisher.publish(msg)
    #     #  self.get_logger().info('Published new sphere position.')

    #     # Update the start position with the random point
    #     planner.start = random_point

    #     # Spin the subscriber to update the current position
    #     rclpy.spin_once(position, timeout_sec=0.1)
    # planner.get_logger().info("Goal reached!")

    position.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # while rclpy.ok():
    #     # Update position
    #     rclpy.spin_once(positionSubscriber.get_current_position(), timeout_sec = 0.1)

    #     current_position = positionSubscriber.get_current_position()
    #     if current_position is not None:
    #         # Update RRT* planner with latest position
    #         planner.start = current_position
