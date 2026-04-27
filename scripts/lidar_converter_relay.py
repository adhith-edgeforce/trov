# #!/usr/bin/env python3
# """
# Relay /points -> /sensing/lidar/concatenated/pointcloud
# with BEST_EFFORT QoS so Autoware's crop_box_filter can receive it.
# """
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import PointCloud2


# class LidarBestEffortRelay(Node):
#     def __init__(self):
#         super().__init__('lidar_besteff_relay')

#         best_effort_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5
#         )

#         self.pub = self.create_publisher(
#             PointCloud2,
#             '/sensing/lidar/concatenated/pointcloud',
#             best_effort_qos
#         )

#         self.sub = self.create_subscription(
#             PointCloud2,
#             '/points_raw',
#             self.callback,
#             10
#         )
#         self.get_logger().info('lidar_besteff_relay started: /points -> /sensing/lidar/concatenated/pointcloud [BEST_EFFORT]')

#     def callback(self, msg):
#         self.pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarBestEffortRelay()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# """
# Convert /points_raw (PointXYZI+ring+time) to PointXYZIRCAEDT
# and publish to /sensing/lidar/concatenated/pointcloud
# """
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs_py.point_cloud2 as pc2
# import numpy as np
# import struct


# class LidarConverterRelay(Node):
#     def __init__(self):
#         super().__init__('lidar_converter_relay')

#         # best_effort_qos = QoSProfile(
#         #     reliability=ReliabilityPolicy.BEST_EFFORT,
#         #     history=HistoryPolicy.KEEP_LAST,
#         #     depth=5
#         # )

#         # self.pub = self.create_publisher(
#         #     PointCloud2,
#         #     '/sensing/lidar/concatenated/pointcloud',
#         #     best_effort_qos
#         # )

#         # self.sub = self.create_subscription(
#         #     PointCloud2,
#         #     '/points_raw',
#         #     self.callback,
#         #     best_effort_qos
#         # )
#         sensor_qos_sub = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,      # must match rslidar_to_lio
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,  # match /points_raw if you checked depth=5
#         )

#         sensor_qos_pub = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,         # for Autoware downstream
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=5,
#         )
#         self.sub = self.create_subscription(
#             PointCloud2,
#             '/points_raw',
#             self.callback,
#             sensor_qos_sub
#         )

#         self.pub = self.create_publisher(
#             PointCloud2,
#             '/sensing/lidar/concatenated/pointcloud',
#             sensor_qos_pub
#         )
#         self.get_logger().info('lidar_converter_relay started')

#     def callback(self, msg):
#         # PointXYZIRCAEDT exact layout from Autoware source:
#         # float x        offset 0   (4 bytes)
#         # float y        offset 4   (4 bytes)
#         # float z        offset 8   (4 bytes)
#         # uint8 intensity offset 12 (1 byte)
#         # uint8 return_type offset 13 (1 byte)
#         # uint16 channel  offset 14 (2 bytes)
#         # float azimuth   offset 16 (4 bytes)
#         # float elevation offset 20 (4 bytes)
#         # float distance  offset 24 (4 bytes)
#         # uint32 time_stamp offset 28 (4 bytes)
#         # total = 32 bytes
#         self.get_logger().info(f'callback fired: {msg.width} points, fields: {[f.name for f in msg.fields]}')
#         self.get_logger().info(f'got msg')
#         #self.pub.publish(msg)

#         fields = [
#             PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
#             PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
#             PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
#             PointField(name='intensity',   offset=12, datatype=PointField.UINT8,   count=1),
#             PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
#             PointField(name='channel',     offset=14, datatype=PointField.UINT16,  count=1),
#             PointField(name='azimuth',     offset=16, datatype=PointField.FLOAT32, count=1),
#             PointField(name='elevation',   offset=20, datatype=PointField.FLOAT32, count=1),
#             PointField(name='distance',    offset=24, datatype=PointField.FLOAT32, count=1),
#             PointField(name='time_stamp',  offset=28, datatype=PointField.UINT32,  count=1),
#         ]
#         # point_step = 32 bytes

#         points = list(pc2.read_points(
#             msg,
#             field_names=['x', 'y', 'z', 'intensity', 'ring', 'time'],
#             skip_nans=True
#         ))

#         if not points:
#             return

#         data = bytearray()
#         for p in points:
#             x         = float(p[0])
#             y         = float(p[1])
#             z         = float(p[2])
#             intensity  = min(int(p[3]), 255)   # clamp to uint8
#             ring       = int(p[4])
#             t          = float(p[5])

#             dist       = float(np.sqrt(x**2 + y**2 + z**2))
#             azimuth    = float(np.arctan2(y, x))
#             elevation  = float(np.arctan2(z, np.sqrt(x**2 + y**2)))
#             return_type = 0
#             channel     = ring  # use ring as channel, uint16
#             time_stamp  = int(t * 1e9) & 0xFFFFFFFF

#             # little-endian:
#             # fff  = x, y, z         (3 x float32)
#             # BB   = intensity, return_type (2 x uint8)
#             # H    = channel         (uint16)
#             # fff  = azimuth, elevation, distance (3 x float32)
#             # I    = time_stamp      (uint32)
#             data += struct.pack('<fffBBHfffI',
#                 x, y, z,
#                 intensity, return_type, channel,
#                 azimuth, elevation, dist,
#                 time_stamp)

#         out = PointCloud2()
#         out.header.frame_id = "lidar_link"
#        out.header.stamp = self.get_clock().now().to_msg()
#         out.height = 1
#         out.width = len(points)
#         out.fields = fields
#         out.is_bigendian = False
#         out.point_step = 32
#         out.row_step = 32 * len(points)
#         out.data = bytes(data)
#         out.is_dense = True

#         self.pub.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarConverterRelay()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Fast vectorized conversion: /points_raw (PointXYZI+ring+time) -> PointXYZIRCAEDT
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


class LidarConverterRelay(Node):
    def __init__(self):
        super().__init__('lidar_converter_relay')

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, '/points_raw', self.callback, sub_qos)
        self.pub = self.create_publisher(
            PointCloud2, '/sensing/lidar/concatenated/pointcloud', pub_qos)

        self.out_fields = [
            PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity',   offset=12, datatype=PointField.UINT8,   count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
            PointField(name='channel',     offset=14, datatype=PointField.UINT16,  count=1),
            PointField(name='azimuth',     offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation',   offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance',    offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='time_stamp',  offset=28, datatype=PointField.UINT32,  count=1),
        ]

        self.get_logger().info('lidar_converter_relay (vectorized) started')

    def callback(self, msg):
        # Parse as structured dtype directly - avoids contiguity issues
        in_dtype = np.dtype([
            ('x',         np.float32),
            ('y',         np.float32),
            ('z',         np.float32),
            ('intensity', np.float32),
            ('ring',      np.uint16),
            ('pad',       np.uint16),
            ('time',      np.float32),
        ])  # 24 bytes total, matches point_step=24

        arr = np.frombuffer(bytes(msg.data), dtype=in_dtype)
        N = arr.shape[0]
        if N == 0:
            return

        x   = arr['x'].copy()
        y   = arr['y'].copy()
        z   = arr['z'].copy()
        i_f = arr['intensity'].copy()
        ring = arr['ring'].copy()
        t_f  = arr['time'].copy()

        # Filter NaNs
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        i_f  = i_f[valid]
        ring = ring[valid]
        t_f  = t_f[valid]
        N = x.shape[0]
        if N == 0:
            return

        # Compute derived fields vectorized
        dist      = np.sqrt(x**2 + y**2 + z**2).astype(np.float32)
        azimuth   = np.arctan2(y, x).astype(np.float32)
        xy        = np.sqrt(x**2 + y**2)
        np.maximum(xy, 1e-6, out=xy)
        elevation = np.arctan2(z, xy).astype(np.float32)

        intensity   = np.clip(i_f, 0, 255).astype(np.uint8)
        return_type = np.zeros(N, dtype=np.uint8)
        channel     = ring.astype(np.uint16)
        time_stamp  = (np.abs(t_f) * 1e9).astype(np.uint32)

        # Build output structured array (32 bytes per point)
        out_dtype = np.dtype([
            ('x',           np.float32),
            ('y',           np.float32),
            ('z',           np.float32),
            ('intensity',   np.uint8),
            ('return_type', np.uint8),
            ('channel',     np.uint16),
            ('azimuth',     np.float32),
            ('elevation',   np.float32),
            ('distance',    np.float32),
            ('time_stamp',  np.uint32),
        ])

        out_arr = np.zeros(N, dtype=out_dtype)
        out_arr['x']           = x
        out_arr['y']           = y
        out_arr['z']           = z
        out_arr['intensity']   = intensity
        out_arr['return_type'] = return_type
        out_arr['channel']     = channel
        out_arr['azimuth']     = azimuth
        out_arr['elevation']   = elevation
        out_arr['distance']    = dist
        out_arr['time_stamp']  = time_stamp

        out = PointCloud2()
        out.header.frame_id = 'lidar_link'
        out.header.stamp = self.get_clock().now().to_msg()
        out.height = 1
        out.width = N
        out.fields = self.out_fields
        out.is_bigendian = False
        out.point_step = 32
        out.row_step = 32 * N
        out.data = out_arr.tobytes()
        out.is_dense = True

        self.pub.publish(out)
        self.get_logger().info(
            f'published {N} pts', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConverterRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()