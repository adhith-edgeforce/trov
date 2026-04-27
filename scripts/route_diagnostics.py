#!/usr/bin/env python3
"""
Route Server Diagnostics
Checks route_server status, graph loading, and helps debug visualization
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import subprocess
import sys

class RouteServerDiagnostics(Node):
    def __init__(self):
        super().__init__('route_server_diagnostics')
        
        # Subscribe to route_graph
        self.graph_sub = self.create_subscription(
            MarkerArray,
            '/route_graph',
            self.graph_callback,
            10
        )
        
        self.graph_received = False
        self.marker_count = 0
        
        self.get_logger().info('Route Server Diagnostics Started')
        self.get_logger().info('Listening to /route_graph topic...')
        
    def graph_callback(self, msg):
        """Callback for route_graph messages"""
        if not self.graph_received:
            self.graph_received = True
            self.marker_count = len(msg.markers)
            
            self.get_logger().info(f'✅ Route graph received! {self.marker_count} markers')
            
            # Analyze markers
            for i, marker in enumerate(msg.markers):
                self.get_logger().info(
                    f'  Marker {i}: type={marker.type}, '
                    f'ns={marker.ns}, id={marker.id}, '
                    f'frame={marker.header.frame_id}'
                )
            
            # Check if markers are valid
            if self.marker_count == 0:
                self.get_logger().warn('⚠️  No markers in route graph!')
                self.get_logger().warn('   Graph might be empty or not loaded')
            else:
                self.get_logger().info(
                    f'\n📍 To visualize in RViz:\n'
                    f'   1. Add > By topic > /route_graph > MarkerArray\n'
                    f'   2. Make sure Fixed Frame is "map"\n'
                    f'   3. Check that markers are enabled\n'
                )

def check_route_server_status():
    """Check route_server lifecycle state"""
    print("\n" + "="*60)
    print("ROUTE SERVER STATUS CHECK")
    print("="*60)
    
    # Check if route_server node exists
    result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    if '/route_server' not in result.stdout:
        print("❌ route_server node NOT running!")
        return False
    print("✅ route_server node is running")
    
    # Check lifecycle state
    result = subprocess.run(['ros2', 'lifecycle', 'get', '/route_server'], 
                          capture_output=True, text=True)
    state = result.stdout.strip()
    print(f"📊 Lifecycle state: {state}")
    
    if 'active' not in state.lower():
        print("⚠️  route_server is not ACTIVE!")
        print("   Try: ros2 lifecycle set /route_server activate")
        return False
    
    print("✅ route_server is ACTIVE")
    
    # Check graph filepath parameter
    result = subprocess.run(
        ['ros2', 'param', 'get', '/route_server', 'GeoJsonGraphFileLoader.graph_filepath'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        graph_path = result.stdout.strip()
        print(f"📁 Graph file path: {graph_path}")
    
    # Check if route_graph topic is publishing
    result = subprocess.run(['ros2', 'topic', 'hz', '/route_graph', '--window', '5'],
                          capture_output=True, text=True, timeout=6)
    if 'average rate' in result.stdout:
        print("✅ /route_graph is publishing")
    else:
        print("⚠️  /route_graph might not be publishing data")
    
    print("="*60 + "\n")
    return True

def main(args=None):
    # First check route_server status
    status_ok = check_route_server_status()
    
    if not status_ok:
        print("Please fix route_server issues before checking graph visualization")
        sys.exit(1)
    
    # Then listen for graph messages
    rclpy.init(args=args)
    
    diagnostics = RouteServerDiagnostics()
    
    print("\n⏳ Waiting for route_graph messages (5 seconds)...")
    
    # Spin for 5 seconds to receive messages
    import time
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 5.0:
        rclpy.spin_once(diagnostics, timeout_sec=0.1)
    
    if not diagnostics.graph_received:
        diagnostics.get_logger().error(
            '\n❌ No route_graph messages received!\n'
            '   Possible causes:\n'
            '   1. Graph file not loaded\n'
            '   2. Graph file is empty\n'
            '   3. route_server not publishing\n'
        )
    
    diagnostics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
