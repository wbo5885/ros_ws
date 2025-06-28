#!/usr/bin/env python3
# Copyright 2024 wb
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Map saving utility for SLAM-generated maps.

This script provides an easy way to save maps created during SLAM sessions
to the maps directory for later use in navigation mode.
"""

import os
import sys
import argparse
from datetime import datetime
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap


class MapSaver(Node):
    """Node for saving SLAM-generated maps."""
    
    def __init__(self):
        super().__init__('map_saver')
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        # Wait for service to be available with better timeout handling
        timeout_counter = 0
        max_timeout_attempts = 30  # 30 seconds total
        while not self.client.wait_for_service(timeout_sec=1.0):
            timeout_counter += 1
            if timeout_counter >= max_timeout_attempts:
                self.get_logger().error('Timed out waiting for slam_toolbox save_map service')
                raise RuntimeError('Service unavailable after 30 seconds')
            self.get_logger().info('Waiting for slam_toolbox save_map service...')
    
    def save_map(self, map_name):
        """Save the current map with specified name."""
        request = SaveMap.Request()
        request.name = map_name
        
        self.get_logger().info(f'Saving map as: {map_name}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Map saved successfully!')
            return True
        else:
            self.get_logger().error('Failed to save map!')
            return False


def main():
    """Main function for map saving script."""
    parser = argparse.ArgumentParser(description='Save SLAM-generated map')
    parser.add_argument(
        'map_name', 
        nargs='?',
        default='my_robot_map',
        help='Name for the saved map (default: my_robot_map)'
    )
    parser.add_argument(
        '--timestamp', 
        action='store_true',
        help='Add timestamp to map name'
    )
    
    args = parser.parse_args()
    
    # Add timestamp if requested
    if args.timestamp:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_name = f"{args.map_name}_{timestamp}"
    else:
        map_name = args.map_name
    
    # Initialize ROS 2
    rclpy.init()
    
    try:
        map_saver = MapSaver()
        success = map_saver.save_map(map_name)
        
        if success:
            print(f"✅ Map '{map_name}' saved successfully!")
            print(f"📁 Check the maps directory for the saved files:")
            print(f"   - {map_name}.yaml")
            print(f"   - {map_name}.pgm")
            print("🚀 You can now use this map for navigation mode!")
        else:
            print("❌ Failed to save map!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("⏹️  Map saving interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        sys.exit(1)
    finally:
        try:
            map_saver.destroy_node()
        except Exception:
            pass  # Node may already be destroyed
        rclpy.shutdown()


if __name__ == '__main__':
    main()