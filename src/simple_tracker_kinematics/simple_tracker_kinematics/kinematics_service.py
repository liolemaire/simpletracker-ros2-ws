import rclpy
from rclpy.node import Node
from simple_tracker_interfaces.srv import PredictCoordinates
import os
import tensorflow as tf
import numpy as np
from ament_index_python.packages import get_package_share_directory



class CoordinatesPredictorServer(Node):

    def __init__(self):
        
        super().__init__('kinematics_service')
        self.srv = self.create_service(PredictCoordinates, 'predict_coordinates', self.handle_predict_coordinates)
        
        # Load the saved model
        
        self.loaded_model = tf.saved_model.load(os.path.join(get_package_share_directory('simple_tracker_kinematics'), "chatgpt.model"))
        self.get_logger().info('Sending back response: distance: %s, ' % (self.loaded_model.signatures["serving_default"].inputs[0].dtype))
        self.signature = self.loaded_model.signatures["serving_default"]

    def handle_predict_coordinates(self, request, response):

        # Prepare the input data for the new coordinates
        # new_x_data = np.array([[request.x, request.y]]).astype(np.float32)
        # if not (min_x <= request.x <= max_x) or not (min_y <= request.y <= max_y):
        #     raise ValueError("Input values are out of range")


        new_x_data = np.array([[request.x, request.y,0,0]]).astype(np.float32)
        
        if new_x_data.shape[1] != 4:
            raise ValueError("Input shape does not match the expected shape of the model")
        

        output = self.signature(tf.constant(new_x_data))
        # Use the model to predict the distance and direction
        
        x_pred, y_pred = output['dense_15'][0].numpy()

    
        self.get_logger().info('Incoming request: x: %d, y: %d' % (request.x, request.y))
        self.get_logger().info('Sending back response: distance: %d, direction: %d' % (x_pred, y_pred))
        response.x_pred = x_pred.astype(float)
        response.y_pred = y_pred.astype(float)
        
        return response

def main(args=None):
    rclpy.init(args=args)

    kinematics_service = CoordinatesPredictorServer()

    rclpy.spin(kinematics_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()