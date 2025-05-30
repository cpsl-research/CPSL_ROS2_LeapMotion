import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import subprocess

class TestRviz(Node):
    '''
    RViz2上でPoseArrayを使用して球体を描画するテストプログラム
    '''
    def __init__(self):
        super().__init__('test_rviz')
        self.pose_array_publisher = self.create_publisher(PoseArray, 'hand_poses', 10)

        # 定期的にPoseArrayを送信
        self.timer = self.create_timer(0.1, self.publish_pose_array)

        # RVizを自動起動
        self.start_rviz()

    def start_rviz(self):
        '''
        RVizを自動的に起動
        '''
        rviz_config_path = 'leap_node/rviz/leapmotion.rviz'
        subprocess.Popen(['rviz2', '-d', rviz_config_path])

    def publish_pose_array(self):
        '''
        PoseArrayを送信
        '''
        pose_array = PoseArray()
        pose_array.header.frame_id = "world"  # フレームID
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # 球体の位置と姿勢を設定
        pose = Pose()
        pose.position.x = 0.2
        pose.position.y = 0.3
        pose.position.z = 0.4
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        # PoseをPoseArrayに追加
        pose_array.poses.append(pose)

        # PoseArrayを送信
        self.pose_array_publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = TestRviz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()