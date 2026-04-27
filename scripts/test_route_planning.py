#!/usr/bin/env python3
"""
Test Route Server Functionality
Even if visualization doesn't work, we can still use route planning!
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ComputeRoute
from geometry_msgs.msg import PoseStamped

class RouteServerTester(Node):
    def __init__(self):
        super().__init__('route_server_tester')
        
        # Create service client
        self.client = self.create_client(ComputeRoute, '/compute_route')
        
    def test_route_planning(self):
        """Test if route planning works even without visualization"""
        
        self.get_logger().info('Testing route planning...')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('compute_route service not available!')
            return False
        
        self.get_logger().info('✅ compute_route service is available')
        
        # Create request - plan from entrance to loading_dock
        request = ComputeRoute.Request()
        
        # Start pose (entrance: 0, 0)
        request.start = PoseStamped()
        request.start.header.frame_id = 'map'
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose.position.x = 0.0
        request.start.pose.position.y = 0.0
        request.start.pose.position.z = 0.0
        request.start.pose.orientation.w = 1.0
        
        # Goal pose (loading_dock: 6, 0)
        request.goal = PoseStamped()
        request.goal.header.frame_id = 'map'
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x = 6.0
        request.goal.pose.position.y = 0.0
        request.goal.pose.position.z = 0.0
        request.goal.pose.orientation.w = 1.0
        
        request.planner_id = 'GridBased'
        request.use_start = True
        
        self.get_logger().info('Calling compute_route service...')
        self.get_logger().info(f'  Start: (0.0, 0.0) - entrance')
        self.get_logger().info(f'  Goal:  (6.0, 0.0) - loading_dock')
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            
            if len(response.path.poses) > 0:
                self.get_logger().info(f'✅ SUCCESS! Route computed with {len(response.path.poses)} waypoints')
                self.get_logger().info('\nWaypoints:')
                for i, pose in enumerate(response.path.poses):
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    self.get_logger().info(f'  {i+1}. ({x:.2f}, {y:.2f})')
                return True
            else:
                self.get_logger().error('❌ Route computed but has no waypoints!')
                return False
        else:
            self.get_logger().error('❌ Service call failed or timed out!')
            return False

def main():
    rclpy.init()
    
    print("\n" + "="*60)
    print("ROUTE SERVER FUNCTIONALITY TEST")
    print("="*60)
    print("\nTesting if route planning works...")
    print("(Visualization might not work, but planning should!)")
    print("")
    
    tester = RouteServerTester()
    
    success = tester.test_route_planning()
    
    print("\n" + "="*60)
    if success:
        print("✅ ROUTE SERVER IS WORKING!")
        print("="*60)
        print("\nRoute planning is functional.")
        print("Visualization issue doesn't affect functionality.")
        print("\nYou can now:")
        print("  1. Use /compute_route to plan paths")
        print("  2. Navigate using /navigate_through_poses")
        print("  3. Build automated warehouse navigation")
        print("")
    else:
        print("❌ ROUTE SERVER NOT WORKING")
        print("="*60)
        print("\nCheck route_server logs for errors")
        print("")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
