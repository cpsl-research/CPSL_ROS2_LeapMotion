import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import leap
import time
import curses

class PostureDetector(Node):
    '''
    Leap Motionから手の位置と姿勢を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self, stdscr):
        super().__init__('posture_detector')
        self.left_hand_publisher = self.create_publisher(PoseStamped, 'left_hand_pose', 10)
        self.right_hand_publisher = self.create_publisher(PoseStamped, 'right_hand_pose', 10)
        self.stdscr = stdscr
        self.listener = LeapMotionListener(self.handle_hand_posture)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

    def handle_hand_posture(self, hands):
        # 検出された手を処理
        self.stdscr.clear()
        for hand in hands:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "leap_frame"
            # 手の位置を設定
            msg.pose.position.x = hand.palm.position.x
            msg.pose.position.y = -hand.palm.position.z
            msg.pose.position.z = hand.palm.position.y
            # 手の姿勢を設定
            msg.pose.orientation.x = hand.palm.orientation.x
            msg.pose.orientation.y = -hand.palm.orientation.z
            msg.pose.orientation.z = hand.palm.orientation.y
            msg.pose.orientation.w = hand.palm.orientation.w

            if str(hand.type) == "HandType.Left":
                self.left_hand_publisher.publish(msg)
                self.stdscr.addstr(f"Left Hand Pose:\n Position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}\n"
                                   f" Orientation: x={msg.pose.orientation.x:.2f}, y={msg.pose.orientation.y:.2f}, z={msg.pose.orientation.z:.2f}, w={msg.pose.orientation.w:.2f}\n"
                                   f" \nframe_id={msg.header.frame_id}, \nstamp={msg.header.stamp}\n\n")
            elif str(hand.type) == "HandType.Right":
                self.right_hand_publisher.publish(msg)
                self.stdscr.addstr(f"Right Hand Pose:\n Position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}\n"
                                   f" Orientation: x={msg.pose.orientation.x:.2f}, y={msg.pose.orientation.y:.2f}, z={msg.pose.orientation.z:.2f}, w={msg.pose.orientation.w:.2f}\n"
                                   f" \nframe_id={msg.header.frame_id}, \nstamp={msg.header.stamp}\n\n")
        self.stdscr.refresh()

    def run(self):
        with self.connection.open():
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while rclpy.ok():
                rclpy.spin_once(self)
                time.sleep(1)

class LeapMotionListener(leap.Listener):
    '''
    Leap Motionのイベント処理を行うクラス
    '''
    def __init__(self, callback):
        super().__init__()
        self.callback = callback  # コールバック関数を保持

    def on_tracking_event(self, event):
        self.callback(event.hands)  # 検出された手をコールバック関数に渡す

def curses_main(stdscr):
    rclpy.init()
    node = PostureDetector(stdscr)
    node.run()
    rclpy.shutdown()

def main(args=None):
    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()