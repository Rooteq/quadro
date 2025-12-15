import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class Imu_viz(Node):
    def __init__(self):
        super().__init__('imu_viz')
        self.imu_sub = self.create_subscription(Imu, 'bno055/imu', self.imu_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)


    def imu_callback(self, msg: Imu):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.tf_broadcaster.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    imu_node = Imu_viz()

    rclpy.spin(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()