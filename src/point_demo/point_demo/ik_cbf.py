#!/usr/bin/env python3
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
import pinocchio as pin
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from pink.visualization import start_meshcat_visualizer
import meshcat_shapes
import meshcat.geometry as g
from pink.configuration import Configuration

# Gazebo ModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


# Might not need
from pink.tasks import PostureTask, FrameTask

class IKCBFNode(Node):
    def __init__(self):
        super().__init__('ik_cbf_node')
        self.get_logger().info('IKCBFNode started')        

        #Debug
        self.test_position = np.array([0.0, 0.0, 0.0])
        
    # Initialize target position attributes as intitial position of the point
        self.x_target = 0
        self.y_target = 0
        self.z_target = 0
        self.get_logger().info('Target position initialized')

        # Create a publisher for point position
        self.point_position_pub = self.create_publisher(Point, '/current_position', 10)
        # self.timer = self.create_timer(0.1, self.publish_current_position)


        urdf_path = os.path.join(get_package_share_directory('point_demo'), 'urdf', 'point.urdf')
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=urdf_path,
            package_dirs=[os.path.join(os.path.dirname(__file__), "urdf")],
            root_joint=None,
        )

        # Visualize robot in meshcat
        self.viz = start_meshcat_visualizer(self.robot)
        self.viewer = self.viz.viewer
        # Set target frame (goal) and tip frame (current)
        meshcat_shapes.frame(self.viewer["target_frame"], opacity = 0.5)
        meshcat_shapes.frame(self.viewer["tip_frame"], opacity = 0.5)

        self.tasks = {
            "tip": FrameTask(
                "point",
                position_cost = 50.0,
                orientation_cost = 20.0,
            ),
            "posture": PostureTask(
                cost = 1.0,
            )
        }

        self.obstacle_position = np.array([3.0, 0.0,0.0])
        obstacle_transform = pin.SE3()
        obstacle_transform.translation = self.obstacle_position
        obstacle_transform.rotation = np.eye(3)
        obstacle_frame = pin.Frame(
            "obstacle",
            0,
            0,
            obstacle_transform,
            type = pin.FrameType.OP_FRAME,
        )
        self.robot.model.addFrame(obstacle_frame)
        self.robot.data = pin.Data(self.robot.model)

        self.point_position = np.array([2.0, 0.0,0.0])
        point_transform = pin.SE3()
        point_transform.translation = self.point_position
        point_transform.rotation = np.eye(3)
        point_frame = pin.Frame(
            "point",
            0,
            0,
            point_transform,
            type = pin.FrameType.OP_FRAME,
        )


        # Set the initial position of the "point" frame in the world frame
        initial_position = np.array([0.0, 0.0, 0.0])
        frame_index = self.robot.model.getFrameId("point")  # Get the frame index
        print("Frame Index:", frame_index)
        self.robot.data.oMf[frame_index] = pin.SE3(np.eye(3), initial_position)  # Set the transformation
        print("Initial Position:", self.robot.data.oMf[frame_index].translation)

        # Visualize obstacle in meshcat
        self.viewer["obstacle"].set_object(g.Sphere(0.15), g.MeshLambertMaterial(color=0x00ff00))
        self.viewer["obstacle"].set_transform(obstacle_transform.np)

        self.configuration = Configuration(self.robot.model, self.robot.data, self.robot.q0)
        for task in self.tasks.values():
            print("Task values: ", task)
            task.set_target_from_configuration(self.configuration)


        # Verify the position
        point_position = self.configuration.get_transform_frame_to_world("point").translation
        #Debug
        print(f"Point position in world frame: {point_position}")
        print([frame.name for frame in self.robot.model.frames])

        # Publishers and Subscribers
        self.create_subscription(Point, "/next_position", self.next_position_callback,10)
        self.get_logger().info('Subscribers and publishers created')
        self.get_logger().info('Model state publisher created')

        self.dt = 1/50.0
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.update)
        self.previous_position = np.zeros(3)

        self.get_logger().info('IKCBF Initialized')

    # def move_sphere(self, x, y, z):
        # # create a client for the SetModelState service
        # client = self.create_client(SetModelState, '/gazebo/set_model_state')

        # # Wait for the service to be available
        # while not client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')

        # # Create the request
        # request = SetModelState.Request()
        # request.model_state.model_name = "point"  # Replace "point" with your sphere's model name
        # request.model_state.pose.position.x = x
        # request.model_state.pose.position.y = y
        # request.model_state.pose.position.z = z
        # request.model_state.pose.orientation.x = 0.0
        # request.model_state.pose.orientation.y = 0.0
        # request.model_state.pose.orientation.z = 0.0
        # request.model_state.pose.orientation.w = 1.0
        # request.model_state.twist.linear.x = 0.0
        # request.model_state.twist.linear.y = 0.0
        # request.model_state.twist.linear.z = 0.0
        # request.model_state.twist.angular.x = 0.0
        # request.model_state.twist.angular.y = 0.0
        # request.model_state.twist.angular.z = 0.0
        # request.model_state.reference_frame = "world"  # Keep the reference frame as "world"

        # # Send the request and wait for the response
        # future = client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        # if future.result() is not None:
        #     self.get_logger().info(f"Moved sphere to position: ({x}, {y}, {z})")
        # else:
        #     self.get_logger().error("Failed to call /gazebo/set_model_state service")


    def next_position_callback(self, msg):
        self.x_target = msg.x
        self.y_target = msg.y
        self.z_target = msg.z
        # print("next_position")
        # print(self.x_target, self.y_target, self.z_target)
    # def publish_current_position(self):
    #     msg = ModelState()
    #     msg.model_name = "point"
    #     msg.pose.position.x = self.x_target
    #     msg.pose.position.y = self.y_target
    #     msg.pose.position.z = self.z_target
    #     self.end_effector_pub.publish(msg)

    def update(self):
        start_timer = time.time()
        point_position = self.configuration.get_transform_frame_to_world("point").translation
        print(point_position)
        position_msg = Point()
        position_msg.x = point_position[0]
        position_msg.y = point_position[1]
        position_msg.z = point_position[2]
    
        # Publish Point message
        self.point_position_pub.publish(position_msg)

        T = self.tasks['tip'].transform_target_to_world

        if(self.t<0.1):
            new_position = point_position
        else:
            new_position = np.array([self.x_target, self.y_target, self.z_target])

        self.test_position[0] = new_position[0]
        self.test_position[1] = new_position[1]
        self.test_position[2] = new_position[2]
        T.translation = self.test_position
        print(T.translation)


        obstacle_transform = pin.SE3()
        obstacle_transform.translation = self.obstacle_position
        obstacle_transform.rotation = np.eye(3)
        obstacle_frame = pin.Frame(
            "obstacle",
            0,
            self.robot.model.getFrameId("point"),
            obstacle_transform,
            type=pin.FrameType.OP_FRAME,
        )
        self.robot.model.addFrame(obstacle_frame)
        self.robot.data = pin.Data(self.robot.model)

        # self.viewer["obstacle"].set_object(g.Sphere(0.15), g.MeshLambertMaterial(color=0x00ff00))  # Green sphere
        # self.viewer["obstacle"].set_transform(obstacle_transform.np)  # Place obstacle in the visualizer

        self.viewer["point"].set_object(g.Sphere(0.15), g.MeshLambertMaterial(color=0xff0000))
        self.viewer["point"].set_transform(T.np)


        # self.viewer["target_frame"].set_transform(T.np)
        # self.viewer["tip_frame"].set_transform(self.configuration.get_transform_frame_to_world("point").np)

        self.t += self.dt
        end_time = time.time()
        execution_time = end_time - start_timer
        # print(f"Execution time: {execution_time:.4f} seconds")   

def main(args=None):
    rclpy.init(args=args)
    node = IKCBFNode()
    # node.move_sphere(1.0,2.0,3.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()