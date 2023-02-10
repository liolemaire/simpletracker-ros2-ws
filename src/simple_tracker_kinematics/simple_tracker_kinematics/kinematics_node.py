import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from simple_tracker_track_provider.tracker import Tracker
# from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray, Detection2D, Detection2DArray
from simple_tracker_interfaces.msg import TrackTrajectoryArray
# from simple_tracker_interfaces.msg import TrackingState, TrackPoint, TrackTrajectory, TrackTrajectoryArray
from simple_tracker_shared.control_loop_node import ConfiguredNode
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from simple_tracker_track_provider.video_tracker import VideoTracker
from rclpy.timer import Timer
from simple_tracker_interfaces.msg import PredictedTrackPoint
import tensorflow as tf
import numpy as np


class SimpleTrackerKinematicsNode(ConfiguredNode):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
      
      super().__init__('simple_tracker_kinematics_node', subscriber_qos_profile)
      
      self.subscription_trajectory_array = self.create_subscription(TrackTrajectoryArray, 'sky360/tracker/trajectory', self.cb_trajectory_array, 10)
  
      self.latest_masked_frame = None
      self.latest_detector_bounding_boxes = None

      self.pub = self.create_publisher(PredictedTrackPoint, 'sky360/tracker/predicted_coordinates',1)
      
      self.loaded_model = tf.saved_model.load("./install/share/simple_tracker_kinematics/chatgpt.model")
      self.signature = self.loaded_model.signatures["serving_default"]

  
  def cb_trajectory_array(self, msg):
      if self.subscription_trajectory_array is None:
        self.get_logger().info("empty trajectory message...")
        return
      else:
        self.latest_trajectory_array = msg
        for trajectory in self.latest_trajectory_array.trajectories:
          trajectory_id = trajectory.id
          trackpoints = trajectory.trajectory
          last_2_trackpoints = trackpoints[-2:]
          
          if self.all_active(last_2_trackpoints):
              
              x = last_2_trackpoints[-1].center.x
              y = last_2_trackpoints[-1].center.y
              
              predicted_point = PredictedTrackPoint()
              predicted_point.header = msg.header
              result_x, result_y = self.handle_predict_coordinates(float(x),float(y))
              predicted_point.center.x = float(result_x)
              predicted_point.center.y = float(result_y)

              self.pub.publish(predicted_point)
      
        return
  
  def all_active(self, last_2_trackpoints):
    active_state = Tracker.ACTIVE_TARGET
    return all(trackpoint.tracking_state == active_state for trackpoint in last_2_trackpoints)


  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()
    self.video_tracker = VideoTracker(self.app_configuration)

  def handle_predict_coordinates(self, x,y):
    # Prepare the input data for the new coordinates
    new_x_data = np.array([[x, y, 0.,0.]]).astype(np.float32)
    output = self.signature(tf.constant(new_x_data))
    tensor = output['dense_15']
    # Use the model to predict the distance and direction
    return_x = tensor.numpy()[0][0]
    return_y = tensor.numpy()[0][1]

    return return_x,return_y


  def config_list(self) -> List[str]:
      return ['tracker_type', 'tracker_detection_sensitivity', 'tracker_active_only', 'tracker_max_active_trackers', 'frame_provider_resize_dimension_h', 
        'frame_provider_resize_dimension_w', 'track_path_plotting_enabled', 'track_prediction_enabled', 'track_validation_enable', 'track_stationary_threshold', 
        'track_orphaned_threshold', 'tracker_min_centre_point_distance_between_bboxes']

  def validate_config(self) -> bool:
    valid = True
    # TODO: This needs to be expanded upon
    if self.app_configuration['tracker_detection_sensitivity'] == None:
      self.get_logger().error('The tracker_detection_sensitivity config entry is null')
      valid = False
      
    return valid


def main(args=None):
    rclpy.init(args=args)

    subscriber_qos_profile = get_topic_subscriber_qos_profile()
    publisher_qos_profile = get_topic_publisher_qos_profile()

    node = SimpleTrackerKinematicsNode(
      subscriber_qos_profile, publisher_qos_profile
    )
   
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

