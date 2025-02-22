# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import sys
from sensor_msgs.msg import Image
from simple_tracker_interfaces.srv import MaskUpdate
import rclpy
from rclpy.node import Node

class MaskClientAsync(Node):

    def __init__(self):
        super().__init__('mask_client_async')
        self.update_client = self.create_client(MaskUpdate, 'sky360/mask/update')
        self.get_logger().info('created mask service update client...')
        while not self.update_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('mask service not available, waiting again...')
        self.request = MaskUpdate.Request()

    def send_request(self, msg_image, mask_type, file_name):
        self.request.file_name = file_name
        self.request.mask_type = mask_type
        self.request.mask = msg_image        
        self.future = self.update_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()