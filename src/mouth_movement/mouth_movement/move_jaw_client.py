import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Float32


class MoveJawClient(Node):

    def __init__(self):
        super().__init__('jaw_client_node')
        # Action client to send goals to jaw controller
        self.jaw_action_client = ActionClient(self, FollowJointTrajectory, '/jaw_controller/follow_joint_trajectory')

        # Subscriber to get data from face tracker
        self.get_data = self.create_subscription(
            Float32,
            'move_jaw',
            self.send_goal,
            10,
        )

    def send_goal(self, position):
        # Message type in JointTrajectory
        goal_msg = JointTrajectory()

        # Joint trajectory points
        jtp = JointTrajectoryPoint()
        jtp.velocities = [0.0]
        jtp.time_from_start.sec = 0
        jtp.time_from_start.nanosec = 0   
        jtp.positions = [position.data]

        # Build message
        goal_msg.points = [jtp]
        goal_msg.joint_names = ["head_jaw_joint"]

        # Assign goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = goal_msg

        self.jaw_action_client.wait_for_server()

        self.get_logger().info('Goal: %f' % jtp.positions[0])

        self.jaw_action_client.send_goal_async(goal)



def main(args=None):
    rclpy.init(args=args)

    move_jaw_client = MoveJawClient()

    rclpy.spin(move_jaw_client)

    move_jaw_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()