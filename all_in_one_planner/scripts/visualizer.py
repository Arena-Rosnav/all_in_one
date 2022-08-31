import geometry_msgs
import numpy as np
import rospy
import std_msgs
import visualization_msgs
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker


class AllInOneVisualizer:

    def __init__(self, model_names: [str], visualize_every_x_iterations: int = 1, visualize_crash: bool = False):

        self.agent_visualization = rospy.Publisher('all_in_one_planner/all_in_one_action_vis', Marker, queue_size=1)

        # if self._evaluation:
        self.agent_visualization_trajectory = rospy.Publisher('all_in_one_planner/all_in_one_action_trajectory_vis',
                                                              Marker,
                                                              queue_size=1)
        self._setup_trajectory_marker()
        self._collisions_markers = []

        self._model_names = model_names
        self.colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0], [1.0, 0.0, 1.0],
                       [0.0, 1.0, 1.0]]  # TODO This is not optimal as only 6 models can be visualized
        # --> Do something like this https://stackoverflow.com/questions/1168260/algorithm-for-generating-unique-colors?

        self._collisions = 0

        self._visualize_every_x_iterations = visualize_every_x_iterations
        self._current_iteration = 0

        self._visualize_crash = visualize_crash

        self._last_action = -1

    def visualize_step(self, action: int, robot_pose: Pose2D):
        # visualize
        if self._current_iteration % self._visualize_every_x_iterations == 0:
            if self._last_action != action:
                self._last_action = action
                self._visualize_action(action)
            self._visualize_episode_actions_as_path(action, robot_pose)
        self._current_iteration += 1

    def reset_visualizer(self):
        self._last_action = -1
        self._action_trajectory_marker.points.clear()
        self._action_trajectory_marker.colors.clear()
        self.agent_visualization_trajectory.publish(self._action_trajectory_marker)

        self._collisions = 0
        self._current_iteration = 0

    def _setup_trajectory_marker(self):
        self._action_trajectory_marker = Marker()
        self._action_trajectory_marker.header.frame_id = 'map'
        self._action_trajectory_marker.id = 1001
        self._action_trajectory_marker.action = visualization_msgs.msg.Marker.ADD
        self._action_trajectory_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        self._action_trajectory_marker.scale.z = 0
        self._action_trajectory_marker.scale.x = 0.1
        self._action_trajectory_marker.scale.y = 0

    def _visualize_episode_actions_as_path(self, action: int, robot_pose: Pose2D):
        # if not in training mode check if robot has been moved
        if len(self._action_trajectory_marker.points) > 0:
            last_pos = np.array(
                [self._action_trajectory_marker.points[-1].x, self._action_trajectory_marker.points[-1].y])
            dist = np.linalg.norm(last_pos - np.array([robot_pose.x, robot_pose.y]))
            if dist > 2:
                self.reset_visualizer()

        marker = self._action_trajectory_marker

        marker.header.stamp = rospy.get_rostime()

        next_point = geometry_msgs.msg.Point()
        next_point.x = robot_pose.x
        next_point.y = robot_pose.y
        next_point.z = 0

        marker.points.append(next_point)

        next_color = std_msgs.msg.ColorRGBA(self.colors[action][0], self.colors[action][1], self.colors[action][2], 1)
        marker.colors.append(next_color)

        self._action_trajectory_marker = marker
        self.agent_visualization_trajectory.publish(self._action_trajectory_marker)

    def _visualize_action(self, action: int):
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = 'map'
        marker.id = 1000

        marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        marker.action = visualization_msgs.msg.Marker.ADD

        marker.color.r = self.colors[action][0]
        marker.color.g = self.colors[action][1]
        marker.color.b = self.colors[action][2]
        marker.color.a = 1

        marker.pose.position.x = -0.5
        marker.pose.position.y = -1.5

        marker.scale.z = 1

        marker.text = self._model_names[action]

        self.agent_visualization.publish(marker)
