#!/usr/bin/env python3
"""
IMU republisher for RViz2.

HoloOcean's IMUSensor publishes sensor_msgs/Imu with frame_id "COM", which
has no TF, so RViz2 cannot display it. This node:

  * broadcasts a static TF  <world_frame> -> <imu_frame>
  * republishes the Imu unchanged except for frame_id = <imu_frame>
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster


class ImuViz(Node):
    def __init__(self):
        super().__init__('imu_viz')

        self.declare_parameter('imu_topic', '/auv0/IMUSensor')
        self.declare_parameter('output_topic', '')         # default: <imu_topic>/rviz
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('imu_frame', 'auv0/imu')

        imu_topic = self.get_parameter('imu_topic').value
        output_topic = self.get_parameter('output_topic').value or f'{imu_topic}/rviz'
        self.world_frame = self.get_parameter('world_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.imu_pub = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self.get_logger().info(
            f'imu_viz: {imu_topic} -> {output_topic}, TF {self.world_frame} -> {self.imu_frame}')

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.imu_frame
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _imu_cb(self, msg: Imu):
        msg.header.frame_id = self.imu_frame
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
