import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import leap
import time
import curses

class GestureDetector(Node):
    '''
    Leap Motionから手のジェスチャー（grab_strengthとpinch_strength）を取得し、ROS2トピックにパブリッシュするクラス
    '''
    def __init__(self, stdscr):
        super().__init__('gesture_detector')
        self.left_hand_publisher = self.create_publisher(String, 'left_hand_gesture', 10)
        self.right_hand_publisher = self.create_publisher(String, 'right_hand_gesture', 10)
        self.stdscr = stdscr
        self.listener = LeapMotionListener(self.handle_hand_gesture)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)

    def handle_hand_gesture(self, hands):
        # 検出された手を処理
        self.stdscr.clear()
        left_hand_detected = False
        right_hand_detected = False

        for hand in hands:
            msg = String()
            grab_strength = hand.grab_strength
            pinch_strength = hand.pinch_strength

            if str(hand.type) == "HandType.Left":
                left_hand_detected = True
                msg.data = f"Grab Strength={grab_strength:.2f}, Pinch Strength={pinch_strength:.2f}"
                self.left_hand_publisher.publish(msg)
                self.stdscr.addstr(f"Left Hand: {msg.data}\n")
            elif str(hand.type) == "HandType.Right":
                right_hand_detected = True
                msg.data = f"Grab Strength={grab_strength:.2f}, Pinch Strength={pinch_strength:.2f}"
                self.right_hand_publisher.publish(msg)
                self.stdscr.addstr(f"Right Hand: {msg.data}\n")

        if not left_hand_detected:
            self.stdscr.addstr("Left hand is not detected\n")
        if not right_hand_detected:
            self.stdscr.addstr("Right hand is not detected\n")

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
    node = GestureDetector(stdscr)
    node.run()
    rclpy.shutdown()

def main(args=None):
    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()